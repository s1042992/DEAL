#include <string.h>
#include <omnetpp.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include <iomanip>
using namespace omnetpp;
using namespace std;
#include "SatMsg_m.h"
#include "densitylevel.h"
/**
 * Derive the Txc1 class from cSimpleModule. In the Tictoc1 network,
 * both the `tic' and `toc' modules are Txc1 objects, created by OMNeT++
 * at the beginning of the simulation.
 */
int arrivalPacketnum = 0;
int generatePacketnum = 0;
double ISLdelay = 0.00003297;
constexpr int MIN = 0;
constexpr int MAX = 10;
#define PI 3.14159265
class Satellite : public cSimpleModule
{
  private:
    simsignal_t arrivalSignal;
    simsignal_t delaySignal;
    simsignal_t congestionSignal;
    simsignal_t batteryEnergySignal;
    simsignal_t queuingSignal;
    simsignal_t queuefull;
  protected:
    // The following redefined virtual function holds the algorithm.
    virtual SatMsg *generateMessage();
    virtual void forwardMessage(SatMsg *msg);
    //virtual void refreshDisplay() const override;

    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    //virtual void finish() override;

    void withoutSplitflow(bool upperFlag, bool lowerFlag, bool rightFlag, bool leftFlag, SatMsg *msg);
    void splitflow(bool upperFlag, bool lowerFlag, bool rightFlag, bool leftFlag, SatMsg *msg);
    void splitflow_X(bool upperFlag, bool lowerFlag, bool rightFlag, bool leftFlag, SatMsg *msg);
    void splitflow_noX(bool upperFlag, bool lowerFlag, bool rightFlag, bool leftFlag, SatMsg *msg);
    int getSatnum(char *satname);
    int getOrbitnum(char *satname);
    int neighborDensity(int localOrbitnum, int localSatnum, string neighbor);
    int getNeighborDensity(string neighbor);
    int MD(char *localSat, char *destSat);
    double fRand(double fMin, double fMax);
    string getUpperNeighbor(int localOrbitnum, int localSatnum);
    string getLowerNeighbor(int localOrbitnum, int localSatnum);
    string getLeftNeighbor(int localOrbitnum, int localSatnum);
    string getRightNeighbor(int localOrbitnum, int localSatnum);
    void TimeinQue(cMessage *msg, int hop);

    float packetCapacity;
    float packetnum;
    int dropPacketTimes;
    volatile float batteryEnergy;
    float initBatteryEnergy;
    float recPower = 0.4;
    float transPower = 0.7;

    double dre;
    double alpha;
    double Xvalue[4];

    simtime_t lastsimTime;
    cQueue queue;
};

// The module class needs to be registered with OMNeT++
Define_Module(Satellite);

void Satellite::initialize()
{
    arrivalSignal = registerSignal("arrival");
    delaySignal = registerSignal("delay");
    congestionSignal = registerSignal("dropmsg");
    batteryEnergySignal = registerSignal("energy");
    queuingSignal = registerSignal("queue");
    queuefull = registerSignal("occupy");

    packetCapacity = 650;
    packetnum = 0.0; //current packet number in queuesss
    dropPacketTimes = 0;
    // Initialize is called at the beginning of the simulation.
    // To bootstrap the tic-toc-tic-toc process, one of the modules needs
    // to send the first message. Let this be `tic'.
    initBatteryEnergy = 5000;
    batteryEnergy = 5000;

    dre = 0.06;
    alpha = 0.5;

    //inital X value
    for(int i = 0; i < 4; i++){
        //i:
        //0: upper X
        //1: lower X
        //2: left X
        //3: right X
        Xvalue[i] = 0.0;
    }
    lastsimTime = simTime();

    int k = intuniform(0,500); // the satellite will send k messages.
    for(int i = 0; i <= k; i++){
        generatePacketnum++;
        SatMsg *msg = generateMessage();
        int r = intuniform(1,20000); // send at different time
        scheduleAt(0.0001 * r , msg);
    }

}

void Satellite::handleMessage(cMessage *msg)
{
    // The handleMessage() method is called whenever a message arrives
    // at the module. Here, we just send it to the other module, through
    // gate `out'. Because both `tic' and `toc' does the same, the message
    // will bounce between the two.
    SatMsg *ttmsg = check_and_cast<SatMsg  *>(msg); //assert
    const char *temp = ttmsg->getDestination();
    //minus recieving power
    batteryEnergy = batteryEnergy - recPower;

    cout<<"packet number in "<< getFullName()<<"'s queue:"<< packetnum <<endl;
    float emptyfullRatio = (packetnum / packetCapacity);
    emit(queuefull, emptyfullRatio);

    packetnum++;
    if(packetnum > packetCapacity){
        drop(ttmsg);
        packetnum--;
        dropPacketTimes++;
        emit(congestionSignal, dropPacketTimes);
        EV << "packet has dropped" << endl;
        cout << "packet has dropped" << endl;
    }

    else if (packetnum <= packetCapacity && strcmp(temp, getFullName()) == 0) {
        // Message arrived.
        int hopcount = ttmsg->getHopCount();
        arrivalPacketnum++;
        // send a signal
        emit(arrivalSignal, hopcount);
        emit(delaySignal,  msg->getArrivalTime() - msg->getCreationTime());
        TimeinQue(msg, hopcount);
        //emit(latencySignal, simTime() - msg->getCreationTime());

        cout<<"latency:"<<msg->getArrivalTime() - msg->getCreationTime()<<endl;
        EV << "Message " << ttmsg << " arrived after " << ttmsg->getHopCount() << " hops.\n";
        bubble("ARRIVED, starting new one!");

        delete ttmsg;

        packetnum--;

    }
    else {
        // We need to forward the message.
        forwardMessage(ttmsg);

    }
}
SatMsg *Satellite::generateMessage()
{
    // Produce source and destination addresses.

    char *src = strdup(getFullName());  // our module index

    int randomorbit = intuniform(1,6);
    int randomsat = intuniform(1,11);
    char stringorbit[50];
    char stringsat[50];
    itoa(randomorbit, stringorbit, 10);
    itoa(randomsat, stringsat, 10);
    char destnode[110] = "";
    strcat(destnode, "sat");
    strcat(destnode, stringorbit);
    strcat(destnode, "_");
    strcat(destnode, stringsat);
    cout<<destnode<<endl;
    int probability = intuniform(0,20);
    int countDensityLevel[6] = {0};

    string densitylevel_1[100];
    string densitylevel_2[100];
    string densitylevel_3[100];
    string densitylevel_4[100];
    string densitylevel_5[100];
    string densitylevel_6[100];
    for(auto it = densityLevel.begin(); it != densityLevel.end(); ++it){
        if(it->second == 6){
            densitylevel_6[countDensityLevel[5]] = it->first;
            countDensityLevel[5]++;
        }
        else if(it->second == 5){
            densitylevel_5[countDensityLevel[4]] = it->first;
            countDensityLevel[4]++;
        }
        else if(it->second == 4){
            densitylevel_4[countDensityLevel[3]] = it->first;
            countDensityLevel[3]++;
        }
        else if(it->second == 3){
            densitylevel_3[countDensityLevel[2]] = it->first;
            countDensityLevel[2]++;
        }
        else if(it->second== 2){
            densitylevel_2[countDensityLevel[1]] = it->first;
            countDensityLevel[1]++;
        }
        else if(it->second == 1){
            densitylevel_1[countDensityLevel[0]] = it->first;
            countDensityLevel[0]++;
        }
    }

    if(probability < 6){
        int temp = intuniform(0,countDensityLevel[5] - 1);
        strcpy(destnode, densitylevel_6[temp].c_str());
    }
    else if(probability >= 6 && probability < 11){
        int temp = intuniform(0,countDensityLevel[4] - 1);
        strcpy(destnode, densitylevel_5[temp].c_str());
    }
    else if(probability >= 11 && probability < 15){
        int temp = intuniform(0,countDensityLevel[3] - 1);
        strcpy(destnode, densitylevel_4[temp].c_str());
    }
    else if(probability >= 15 && probability < 18){
        int temp = intuniform(0,countDensityLevel[2] - 1);
        strcpy(destnode, densitylevel_3[temp].c_str());
    }
    else if(probability >= 18 && probability < 20){
        int temp = intuniform(0,countDensityLevel[1] - 1);
        strcpy(destnode, densitylevel_2[temp].c_str());
    }

    //char dest[] = "sat5_7";
    char msgname[20];
    sprintf(msgname, "%d-to-%d", src, destnode);

    // Create message object and set source and destination field.
    SatMsg *msg = new SatMsg(msgname);
    msg->setSource(src);
    msg->setDestination(destnode);
    //msg->setBitLength(intuniform(512,1024)); //the range of packetlength is from 512b to 1024b
    msg->setByteLength(512);
    return msg;

}

void Satellite::forwardMessage(SatMsg *msg)
{
    // In this example, we just pick a random gate to send it on.
    // We draw a random number between 0 and the size of gate `out[]'.
    char *local = strdup(getFullName()); //local orbit number
    char *dest = strdup(msg->getDestination()); //destination orbit number

    int localOrbitnum = getOrbitnum(local);
    int localSatnum = getSatnum(local);

    cout<<"local node is:" <<localOrbitnum <<"_"<<localSatnum<<endl;

    int destOrbitnum = getOrbitnum(dest);
    int destSatnum = getSatnum(dest);

    cout<<"destination node is: " <<destOrbitnum <<"_"<<destSatnum<<endl;


    //interface(gate) of satellite, "true" means open this interface, and default is "false".
    bool lowerFlag = false;
    bool upperFlag = false;
    bool leftFlag = false;
    bool rightFlag = false;

    // determine the upper or lower gate
    if(abs(destSatnum - localSatnum) > floor(11/2)){
        if(destSatnum > localSatnum){
            upperFlag = true;
        }
        else
            lowerFlag = true;
    }

    else if(abs(destSatnum - localSatnum) <= floor(11/2)){
        if(destSatnum > localSatnum){
            lowerFlag = true;
        }
        else
            upperFlag = true;
    }


    //determine the left or right gate
    if(localOrbitnum > destOrbitnum){
        leftFlag = true;
    }
    else if(localOrbitnum < destOrbitnum){
        rightFlag = true;
    }

    if(destSatnum == localSatnum){
        if(localOrbitnum > destOrbitnum){
            leftFlag = true;
            lowerFlag = false;
            upperFlag = false;
            rightFlag = false;
        }
        else if(localOrbitnum < destOrbitnum){
            leftFlag = false;
            lowerFlag = false;
            upperFlag = false;
            rightFlag = true;
        }
    }

    //withoutSplitflow(upperFlag, lowerFlag, rightFlag, leftFlag, msg);
    //splitflow(upperFlag, lowerFlag, rightFlag, leftFlag, msg);
    splitflow_X(upperFlag, lowerFlag, rightFlag, leftFlag, msg);
    //splitflow_noX(upperFlag, lowerFlag, rightFlag, leftFlag, msg);

    if(simTime() - lastsimTime > dre){
        for(int i = 0; i < 4; i++){
            Xvalue[i] = Xvalue[i] * (1 - alpha);
            cout << "i: " << Xvalue[i] << endl;
        }
    }
    lastsimTime = simTime();
}
void Satellite::finish(){
    cout<<"Arrival Packet number: "<<arrivalPacketnum<<endl;
    cout<<"Generated Packet number: "<<generatePacketnum<<endl;
    cout<<"Packet delivery ratio: " <<arrivalPacketnum*100/generatePacketnum<<"%"<<endl;
}
string Satellite::getUpperNeighbor(int localOrbitnum, int localSatnum){
    int upperneighborOrbitnum = localOrbitnum;
    int upperneighborSatnum;
    if(localSatnum == 1)
        upperneighborSatnum = 11;
    else
        upperneighborSatnum = localSatnum - 1;

    string upperorbit,uppersat;
    stringstream  ssupperorbit, ssuppersat;
    ssupperorbit<<upperneighborOrbitnum;
    ssupperorbit>>upperorbit;
    ssuppersat<<upperneighborSatnum;
    ssuppersat>>uppersat;
    string upperneighbor = "sat" + upperorbit + "_" + uppersat;
    return upperneighbor;
}
string Satellite::getLowerNeighbor(int localOrbitnum, int localSatnum){
    int lowerneighborSatnum;
    int lowerneighborOrbitnum = localOrbitnum;
    if(localSatnum == 11)
        lowerneighborSatnum = 1;
    else
        lowerneighborSatnum = localSatnum + 1;

    string lowerorbit, lowersat;
    stringstream  sslowerorbit, sslowersat;
    sslowerorbit<<lowerneighborOrbitnum;
    sslowerorbit>>lowerorbit;
    sslowersat<<lowerneighborSatnum;
    sslowersat>>lowersat;
    string lowerneighbor = "sat" + lowerorbit + "_" + lowersat;
    return lowerneighbor;
}

string Satellite::getLeftNeighbor(int localOrbitnum, int localSatnum){
    int leftneighborOrbitnum;
    int leftneighborSatnum = localSatnum;
    if(localOrbitnum == 1)
        leftneighborOrbitnum = 6;
    else
        leftneighborOrbitnum = localOrbitnum - 1;
    string leftorbit, leftsat;
    stringstream  ssleftorbit, ssleftsat;
    ssleftorbit<<leftneighborOrbitnum;
    ssleftorbit>>leftorbit;
    ssleftsat<<leftneighborSatnum;
    ssleftsat>>leftsat;
    string leftneighbor = "sat" + leftorbit + "_" + leftsat;
    return leftneighbor;
}

string Satellite::getRightNeighbor(int localOrbitnum, int localSatnum){
    int rightneighborOrbitnum;
    int rightneighborSatnum = localSatnum;
    if(localOrbitnum == 6)
       rightneighborOrbitnum = 1;
    else
       rightneighborOrbitnum = localOrbitnum + 1;
    string rightorbit, rightsat;
    stringstream  ssrightorbit, ssrightsat;
    ssrightorbit<<rightneighborOrbitnum;
    ssrightorbit>>rightorbit;
    ssrightsat<<rightneighborSatnum;
    ssrightsat>>rightsat;
    string rightneighbor = "sat" + rightorbit + "_" + rightsat;
    return rightneighbor;
}
int Satellite::neighborDensity(int localOrbitnum, int localSatnum, string neighbor){


    int upperdensity = getNeighborDensity(getUpperNeighbor(localOrbitnum,localSatnum) );
    int lowerdensity = getNeighborDensity(getLowerNeighbor(localOrbitnum,localSatnum) );
    int leftdensity = getNeighborDensity(getLeftNeighbor(localOrbitnum,localSatnum) );
    int rightdensity = getNeighborDensity(getRightNeighbor(localOrbitnum,localSatnum) );

    if(neighbor == "upper")
        return upperdensity;
    else if (neighbor == "lower")
        return lowerdensity;
    else if (neighbor == "left")
        return leftdensity;
    else if (neighbor == "right")
        return rightdensity;
}
int Satellite::getNeighborDensity(string neighbor){
    return densityLevel[neighbor];
}

void Satellite::splitflow(bool upperFlag, bool lowerFlag, bool rightFlag, bool leftFlag, SatMsg *msg){
    char *local = strdup(getFullName());
    int localOrbitnum = getOrbitnum(local);
    int localSatnum = getSatnum(local);

    int upperdensity = neighborDensity(localOrbitnum, localSatnum, "upper");
    int rightdensity = neighborDensity(localOrbitnum, localSatnum, "right");
    int leftdensity = neighborDensity(localOrbitnum, localSatnum, "left");
    int lowerdensity = neighborDensity(localOrbitnum, localSatnum, "lower");
    int densityRandomrange, temp;
    double delay = 0.0002;
    //split flow
    if(rightFlag && upperFlag){
        densityRandomrange = rightdensity + upperdensity;
        temp = intuniform(1,densityRandomrange);
        if(rightdensity > upperdensity){
            if(temp > upperdensity){
                cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
                simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
                if(rightChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"right$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[3]++;
                }
            }
            else{
                cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
                simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
                if(upperChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"upper$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[0]++;
                }
            }

        }
        else{
            if(temp > rightdensity){
                cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
                simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
                if(upperChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"upper$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[0]++;
                }
            }
            else{
                cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
                simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
                if(rightChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"right$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[3]++;
                }
            }
        }
    }
    else if(rightFlag && lowerFlag){
        densityRandomrange = rightdensity + lowerdensity;
        temp = intuniform(1,densityRandomrange);
        if(rightdensity > lowerdensity){
            if(temp > lowerdensity){
                cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
                simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
                if(rightChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"right$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[3]++;
                }
            }
            else{
                cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
                simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
                if(lowerChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"lower$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[1]++;
                }
            }
        }
        else{
            if(temp > rightdensity){
                cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
                simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
                if(lowerChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"lower$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[1]++;
                }
            }
            else{
                cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
                simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
                if(rightChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"right$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[3]++;
                }
            }
        }
    }
    else if(leftFlag && upperFlag){
        densityRandomrange = leftdensity + upperdensity;
        temp = intuniform(1,densityRandomrange);
        if(leftdensity > upperdensity){
            if(temp > upperdensity){
                cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
                simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
                if(leftChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"left$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[2]++;
                }
            }
            else{
                cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
                simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
                if(upperChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"upper$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[0]++;
                }
            }
        }
        else{
            if(temp > leftdensity){
                cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
                simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
                if(upperChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"upper$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[0]++;
                }
            }
            else{
                cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
                simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
                if(leftChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"left$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[2]++;
                }
            }
        }
    }
    else if(leftFlag && lowerFlag){
        densityRandomrange = leftdensity + lowerdensity;
        temp = intuniform(1,densityRandomrange);
        if(leftdensity > lowerdensity){
            if(temp > lowerdensity){
                cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
                simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
                if(leftChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"left$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[2]++;
                }
            }
            else{
                cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
                simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
                if(lowerChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"lower$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[1]++;
                }
            }
        }
        else{
            if(temp > leftdensity){
                cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
                simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
                if(lowerChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"lower$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[1]++;
                }
            }
            else{
                cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
                simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
                if(leftChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"left$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[2]++;
                }
            }
        }
    }
    else if(rightFlag && !leftFlag && !upperFlag && !lowerFlag){
        if(localOrbitnum != 6){

            cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
            simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
            if(rightChannel->isBusy()){
                cout<<"waiting for transmit"<<endl;
                scheduleAt(simTime()+delay,msg);
            }
            else{
                msg->setHopCount(msg->getHopCount()+1);
                send(msg,"right$o");
                batteryEnergy = batteryEnergy - transPower;
                packetnum--;
                Xvalue[3]++;
            }
        }
    }
    else if(!rightFlag && leftFlag && !upperFlag && !lowerFlag){
        if(localOrbitnum != 1){

            cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
            simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
            if(leftChannel->isBusy()){
                cout<<"waiting for transmit"<<endl;
                scheduleAt(simTime()+delay,msg);
            }
            else{
                msg->setHopCount(msg->getHopCount()+1);
                send(msg,"left$o");
                batteryEnergy = batteryEnergy - transPower;
                packetnum--;
                Xvalue[2]++;
            }
        }
    }
    else if(!rightFlag && !leftFlag && upperFlag && !lowerFlag){

        cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
        simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
        if(upperChannel->isBusy()){
            cout<<"waiting for transmit"<<endl;
            scheduleAt(simTime()+delay,msg);
        }
        else{
            msg->setHopCount(msg->getHopCount()+1);
            send(msg,"upper$o");
            batteryEnergy = batteryEnergy - transPower;
            packetnum--;
            Xvalue[0]++;
        }
    }
    else if(!rightFlag && !leftFlag && !upperFlag && lowerFlag){

        cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
        simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
        if(lowerChannel->isBusy()){
            cout<<"waiting for transmit"<<endl;
            scheduleAt(simTime()+delay,msg);
        }
        else{
            msg->setHopCount(msg->getHopCount()+1);
            send(msg,"lower$o");
            batteryEnergy = batteryEnergy - transPower;
            packetnum--;
            Xvalue[1]++;
        }
    }
    emit(batteryEnergySignal, batteryEnergy);
}

void Satellite::splitflow_X(bool upperFlag, bool lowerFlag, bool rightFlag, bool leftFlag, SatMsg *msg){
    char *local = strdup(getFullName());
    char *dest = strdup(msg->getDestination());
    int localOrbitnum = getOrbitnum(local);
    int localSatnum = getSatnum(local);
    int MDDistance = MD(local, dest);

    double upper_congestion_level, lower_congestion_level, left_congestion_level, right_congestion_level, k, energyburden;
    double cost[4], energy[4];
    k = 1 - (MDDistance / 10);
    //k = 1;

    //to acquire other satellite's energy information
    cModule *uppercalleeModule = getParentModule()->getSubmodule(getUpperNeighbor(localOrbitnum, localSatnum).c_str());
    Satellite *uppercallee = check_and_cast<Satellite *>(uppercalleeModule);
    energy[0] = uppercallee->batteryEnergy / initBatteryEnergy;

    cModule *lowercalleeModule = getParentModule()->getSubmodule(getLowerNeighbor(localOrbitnum, localSatnum).c_str());
    Satellite *lowercallee = check_and_cast<Satellite *>(lowercalleeModule);
    energy[1] = lowercallee->batteryEnergy / initBatteryEnergy;

    cModule *leftcalleeModule = getParentModule()->getSubmodule(getLeftNeighbor(localOrbitnum, localSatnum).c_str());
    Satellite *leftcallee = check_and_cast<Satellite *>(leftcalleeModule);
    energy[2] = leftcallee->batteryEnergy / initBatteryEnergy;

    cModule *rightcalleeModule = getParentModule()->getSubmodule(getRightNeighbor(localOrbitnum, localSatnum).c_str());
    Satellite *rightcallee = check_and_cast<Satellite *>(rightcalleeModule);
    energy[3] = rightcallee->batteryEnergy / initBatteryEnergy;


    double densityRandomrange, temp;
    double delay = 0.0002;
    for(auto it = densityLevel.begin(); it != densityLevel.end(); ++it){
        if(it->first == getUpperNeighbor(localOrbitnum, localSatnum)){
            upper_congestion_level = (atan(Xvalue[0] * (it->second)) * 2) / PI;
            cost[0] = upper_congestion_level + energy[0] * k;
        }
        else if(it->first == getLowerNeighbor(localOrbitnum, localSatnum)){
            lower_congestion_level = (atan(Xvalue[1] * (it->second)) * 2) / PI;
            cost[1] = lower_congestion_level + energy[1] * k;
        }
        else if(it->first == getLeftNeighbor(localOrbitnum, localSatnum)){
            left_congestion_level = (atan(Xvalue[2] * (it->second)) * 2) / PI;
            cost[2] = left_congestion_level + energy[2] * k;
        }
        else if(it->first == getRightNeighbor(localOrbitnum, localSatnum)){
            right_congestion_level = (atan(Xvalue[3] * (it->second)) * 2) / PI;
            cost[3] = right_congestion_level + energy[3] * k;
        }
    }



    //split flow
    if(rightFlag && upperFlag){
        densityRandomrange =  cost[3] + cost[0];
        //temp = intuniform(1,densityRandomrange);
        temp = fRand(0, densityRandomrange);
        if( cost[3] > cost[0]){
            if(temp > cost[0]){
                cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
                simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
                if(rightChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"right$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[3]++;
                }
            }
            else{
                cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
                simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
                if(upperChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"upper$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[0]++;
                }
            }

        }
        else{
            if(temp > cost[3]){
                cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
                simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
                if(upperChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"upper$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[0]++;
                }
            }
            else{
                cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
                simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
                if(rightChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"right$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[3]++;
                }
            }
        }
    }
    else if(rightFlag && lowerFlag){
        densityRandomrange = cost[3] + cost[1];
        temp = intuniform(1,densityRandomrange);
        if(cost[3] > cost[1]){
            if(temp > cost[1]){
                cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
                simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
                if(rightChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"right$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[3]++;
                }
            }
            else{
                cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
                simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
                if(lowerChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"lower$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[1]++;
                }
            }
        }
        else{
            if(temp > cost[3]){
                cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
                simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
                if(lowerChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"lower$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[1]++;
                }
            }
            else{
                cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
                simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
                if(rightChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"right$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[3]++;
                }
            }
        }
    }
    else if(leftFlag && upperFlag){
        densityRandomrange = cost[2] + cost[0];
        temp = intuniform(1,densityRandomrange);
        if(cost[2] > cost[0]){
            if(temp > cost[0]){
                cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
                simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
                if(leftChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"left$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[2]++;
                }
            }
            else{
                cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
                simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
                if(upperChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"upper$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[0]++;
                }
            }
        }
        else{
            if(temp > cost[2]){
                cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
                simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
                if(upperChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"upper$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[0]++;
                }
            }
            else{
                cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
                simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
                if(leftChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"left$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[2]++;
                }
            }
        }
    }
    else if(leftFlag && lowerFlag){
        densityRandomrange =  cost[2]+  cost[1];
        temp = intuniform(1,densityRandomrange);
        if( cost[2] >  cost[1]){
            if(temp > lower_congestion_level){
                cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
                simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
                if(leftChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"left$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[2]++;
                }
            }
            else{
                cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
                simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
                if(lowerChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"lower$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[1]++;
                }
            }
        }
        else{
            if(temp > cost[1]){
                cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
                simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
                if(lowerChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"lower$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[1]++;
                }
            }
            else{
                cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
                simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
                if(leftChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"left$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[2]++;
                }
            }
        }
    }
    else if(rightFlag && !leftFlag && !upperFlag && !lowerFlag){
        if(localOrbitnum != 6){

            cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
            simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
            if(rightChannel->isBusy()){
                cout<<"waiting for transmit"<<endl;
                scheduleAt(simTime()+delay,msg);
            }
            else{
                msg->setHopCount(msg->getHopCount()+1);
                send(msg,"right$o");
                batteryEnergy = batteryEnergy - transPower;
                packetnum--;
                Xvalue[3]++;
            }
        }
    }
    else if(!rightFlag && leftFlag && !upperFlag && !lowerFlag){
        if(localOrbitnum != 1){

            cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
            simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
            if(leftChannel->isBusy()){
                cout<<"waiting for transmit"<<endl;
                scheduleAt(simTime()+delay,msg);
            }
            else{
                msg->setHopCount(msg->getHopCount()+1);
                send(msg,"left$o");
                batteryEnergy = batteryEnergy - transPower;
                packetnum--;
                Xvalue[2]++;
            }
        }
    }
    else if(!rightFlag && !leftFlag && upperFlag && !lowerFlag){

        cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
        simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
        if(upperChannel->isBusy()){
            cout<<"waiting for transmit"<<endl;
            scheduleAt(simTime()+delay,msg);
        }
        else{
            msg->setHopCount(msg->getHopCount()+1);
            send(msg,"upper$o");
            batteryEnergy = batteryEnergy - transPower;
            packetnum--;
            Xvalue[0]++;
        }
    }
    else if(!rightFlag && !leftFlag && !upperFlag && lowerFlag){

        cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
        simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
        if(lowerChannel->isBusy()){
            cout<<"waiting for transmit"<<endl;
            scheduleAt(simTime()+delay,msg);
        }
        else{
            msg->setHopCount(msg->getHopCount()+1);
            send(msg,"lower$o");
            batteryEnergy = batteryEnergy - transPower;
            packetnum--;
            Xvalue[1]++;
        }
    }
    emit(batteryEnergySignal, batteryEnergy);
}

void Satellite::splitflow_noX(bool upperFlag, bool lowerFlag, bool rightFlag, bool leftFlag, SatMsg *msg){
    char *local = strdup(getFullName());
    char *dest = strdup(msg->getDestination());
    int localOrbitnum = getOrbitnum(local);
    int localSatnum = getSatnum(local);
    int MDDistance = MD(local, dest);

    double upper_congestion_level, lower_congestion_level, left_congestion_level, right_congestion_level, k, energyburden;
    double cost[4], energy[4];
    k = 1 - (MDDistance / 10);

    cModule *uppercalleeModule = getParentModule()->getSubmodule(getUpperNeighbor(localOrbitnum, localSatnum).c_str());
    Satellite *uppercallee = check_and_cast<Satellite *>(uppercalleeModule);
    energy[0] = uppercallee->batteryEnergy;

    cModule *lowercalleeModule = getParentModule()->getSubmodule(getLowerNeighbor(localOrbitnum, localSatnum).c_str());
    Satellite *lowercallee = check_and_cast<Satellite *>(lowercalleeModule);
    energy[1] = lowercallee->batteryEnergy;

    cModule *leftcalleeModule = getParentModule()->getSubmodule(getLeftNeighbor(localOrbitnum, localSatnum).c_str());
    Satellite *leftcallee = check_and_cast<Satellite *>(leftcalleeModule);
    energy[2] = leftcallee->batteryEnergy;

    cModule *rightcalleeModule = getParentModule()->getSubmodule(getRightNeighbor(localOrbitnum, localSatnum).c_str());
    Satellite *rightcallee = check_and_cast<Satellite *>(rightcalleeModule);
    energy[3] = rightcallee->batteryEnergy;


    double densityRandomrange, temp;
    double delay = 0.0002;
    for(auto it = densityLevel.begin(); it != densityLevel.end(); ++it){
        if(it->first == getUpperNeighbor(localOrbitnum, localSatnum)){
            upper_congestion_level =  (it->second);
            cost[0] = upper_congestion_level + energy[0] * k;
        }
        else if(it->first == getLowerNeighbor(localOrbitnum, localSatnum)){
            lower_congestion_level = (it->second);
            cost[1] = lower_congestion_level + energy[1] * k;
        }
        else if(it->first == getLeftNeighbor(localOrbitnum, localSatnum)){
            left_congestion_level = (it->second);
            cost[2] = left_congestion_level + energy[2] * k;
        }
        else if(it->first == getRightNeighbor(localOrbitnum, localSatnum)){
            right_congestion_level =  (it->second);
            cost[3] = right_congestion_level + energy[3] * k;
        }
    }



    //split flow
    if(rightFlag && upperFlag){
        densityRandomrange =  cost[3] + cost[0];
        //temp = intuniform(1,densityRandomrange);
        temp = fRand(0, densityRandomrange);
        if( cost[3] > cost[0]){
            if(temp > cost[0]){
                cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
                simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
                if(rightChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"right$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[3]++;
                }
            }
            else{
                cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
                simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
                if(upperChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"upper$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[0]++;
                }
            }

        }
        else{
            if(temp > cost[3]){
                cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
                simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
                if(upperChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"upper$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[0]++;
                }
            }
            else{
                cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
                simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
                if(rightChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"right$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[3]++;
                }
            }
        }
    }
    else if(rightFlag && lowerFlag){
        densityRandomrange = cost[3] + cost[1];
        temp = intuniform(1,densityRandomrange);
        if(cost[3] > cost[1]){
            if(temp > cost[1]){
                cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
                simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
                if(rightChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"right$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[3]++;
                }
            }
            else{
                cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
                simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
                if(lowerChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"lower$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[1]++;
                }
            }
        }
        else{
            if(temp > cost[3]){
                cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
                simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
                if(lowerChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"lower$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[1]++;
                }
            }
            else{
                cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
                simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
                if(rightChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"right$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[3]++;
                }
            }
        }
    }
    else if(leftFlag && upperFlag){
        densityRandomrange = cost[2] + cost[0];
        temp = intuniform(1,densityRandomrange);
        if(cost[2] > cost[0]){
            if(temp > cost[0]){
                cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
                simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
                if(leftChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"left$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[2]++;
                }
            }
            else{
                cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
                simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
                if(upperChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"upper$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[0]++;
                }
            }
        }
        else{
            if(temp > cost[2]){
                cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
                simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
                if(upperChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"upper$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[0]++;
                }
            }
            else{
                cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
                simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
                if(leftChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"left$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[2]++;
                }
            }
        }
    }
    else if(leftFlag && lowerFlag){
        densityRandomrange =  cost[2]+  cost[1];
        temp = intuniform(1,densityRandomrange);
        if( cost[2] >  cost[1]){
            if(temp > lower_congestion_level){
                cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
                simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
                if(leftChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"left$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[2]++;
                }
            }
            else{
                cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
                simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
                if(lowerChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"lower$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[1]++;
                }
            }
        }
        else{
            if(temp > cost[1]){
                cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
                simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
                if(lowerChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"lower$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[1]++;
                }
            }
            else{
                cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
                simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
                if(leftChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"left$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                    Xvalue[2]++;
                }
            }
        }
    }
    else if(rightFlag && !leftFlag && !upperFlag && !lowerFlag){
        if(localOrbitnum != 6){

            cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
            simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
            if(rightChannel->isBusy()){
                cout<<"waiting for transmit"<<endl;
                scheduleAt(simTime()+delay,msg);
            }
            else{
                msg->setHopCount(msg->getHopCount()+1);
                send(msg,"right$o");
                batteryEnergy = batteryEnergy - transPower;
                packetnum--;
                Xvalue[3]++;
            }
        }
    }
    else if(!rightFlag && leftFlag && !upperFlag && !lowerFlag){
        if(localOrbitnum != 1){

            cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
            simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
            if(leftChannel->isBusy()){
                cout<<"waiting for transmit"<<endl;
                scheduleAt(simTime()+delay,msg);
            }
            else{
                msg->setHopCount(msg->getHopCount()+1);
                send(msg,"left$o");
                batteryEnergy = batteryEnergy - transPower;
                packetnum--;
                Xvalue[2]++;
            }
        }
    }
    else if(!rightFlag && !leftFlag && upperFlag && !lowerFlag){

        cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
        simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
        if(upperChannel->isBusy()){
            cout<<"waiting for transmit"<<endl;
            scheduleAt(simTime()+delay,msg);
        }
        else{
            msg->setHopCount(msg->getHopCount()+1);
            send(msg,"upper$o");
            batteryEnergy = batteryEnergy - transPower;
            packetnum--;
            Xvalue[0]++;
        }
    }
    else if(!rightFlag && !leftFlag && !upperFlag && lowerFlag){

        cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
        simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
        if(lowerChannel->isBusy()){
            cout<<"waiting for transmit"<<endl;
            scheduleAt(simTime()+delay,msg);
        }
        else{
            msg->setHopCount(msg->getHopCount()+1);
            send(msg,"lower$o");
            batteryEnergy = batteryEnergy - transPower;
            packetnum--;
            Xvalue[1]++;
        }
    }
    emit(batteryEnergySignal, batteryEnergy);
}
void Satellite::withoutSplitflow(bool upperFlag, bool lowerFlag, bool rightFlag, bool leftFlag, SatMsg *msg){
    // without split flow
    char *local = strdup(getFullName());
    int localOrbitnum = getOrbitnum(local);
    int localSatnum = getSatnum(local);
    double delay = 0.0002;
    if(rightFlag && upperFlag){

            cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
            simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
            if(upperChannel->isBusy()){
                cout<<"waiting for transmit"<<endl;
                scheduleAt(simTime()+delay,msg);
            }
            else{
                msg->setHopCount(msg->getHopCount()+1);
                send(msg,"upper$o");
                batteryEnergy = batteryEnergy - transPower;
                packetnum--;
            }
        }
        else if(rightFlag && lowerFlag){

            cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
            simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
            if(lowerChannel->isBusy()){
                cout<<"waiting for transmit"<<endl;
                scheduleAt(simTime()+delay,msg);
            }
            else{
                msg->setHopCount(msg->getHopCount()+1);
                send(msg,"lower$o");
                batteryEnergy = batteryEnergy - transPower;
                packetnum--;
            }
        }
        else if(leftFlag && upperFlag){

            cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
            simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
            if(upperChannel->isBusy()){
                cout<<"waiting for transmit"<<endl;
                scheduleAt(simTime()+delay,msg);
            }
            else{
                msg->setHopCount(msg->getHopCount()+1);
                send(msg,"upper$o");
                batteryEnergy = batteryEnergy - transPower;
                packetnum--;
            }
        }
        else if(leftFlag && lowerFlag){

            cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
            simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();

            if(lowerChannel->isBusy()){
                cout<<"waiting for transmit"<<endl;
                scheduleAt(simTime()+delay,msg);
            }
            else{
                msg->setHopCount(msg->getHopCount()+1);
                send(msg,"lower$o");
                batteryEnergy = batteryEnergy - transPower;
                packetnum--;
            }
        }
        else if(rightFlag && !leftFlag && !upperFlag && !lowerFlag){
            if(localOrbitnum != 6){

                cChannel *rightChannel = gate("right$o")->getTransmissionChannel();
                simtime_t rightFinishTime = rightChannel->getTransmissionFinishTime();
                if(rightChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"right$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                }
            }
        }
        else if(!rightFlag && leftFlag && !upperFlag && !lowerFlag){
            if(localOrbitnum != 1){

                cChannel *leftChannel = gate("left$o")->getTransmissionChannel();
                simtime_t leftFinishTime = leftChannel->getTransmissionFinishTime();
                if(leftChannel->isBusy()){
                    cout<<"waiting for transmit"<<endl;
                    scheduleAt(simTime()+delay,msg);
                }
                else{
                    msg->setHopCount(msg->getHopCount()+1);
                    send(msg,"left$o");
                    batteryEnergy = batteryEnergy - transPower;
                    packetnum--;
                }
            }
        }
        else if(!rightFlag && !leftFlag && upperFlag && !lowerFlag){

            cChannel *upperChannel = gate("upper$o")->getTransmissionChannel();
            simtime_t upperFinishTime = upperChannel->getTransmissionFinishTime();
            if(upperChannel->isBusy()){
                cout<<"waiting for transmit"<<endl;
                scheduleAt(simTime()+delay,msg);
            }
            else{
                msg->setHopCount(msg->getHopCount()+1);
                send(msg,"upper$o");
                batteryEnergy = batteryEnergy - transPower;
                packetnum--;
            }
        }
        else if(!rightFlag && !leftFlag && !upperFlag && lowerFlag){

            cChannel *lowerChannel = gate("lower$o")->getTransmissionChannel();
            simtime_t lowerFinishTime = lowerChannel->getTransmissionFinishTime();
            if(lowerChannel->isBusy()){
                cout<<"waiting for transmit"<<endl;
                scheduleAt(simTime()+delay,msg);
            }
            else{
                msg->setHopCount(msg->getHopCount()+1);
                send(msg,"lower$o");
                batteryEnergy = batteryEnergy - transPower;
                packetnum--;
            }
        }
        emit(batteryEnergySignal, batteryEnergy);
}

int Satellite::getOrbitnum(char *satname){
    int orbitnum = (int)satname[3] - 48; //orbit number
    return orbitnum;
}
int Satellite::getSatnum(char *satname){
    int satnum;
    if(satname[6] >= '0' && satname[6] <= '9' ){
        satnum = ((int)satname[5] - 48) * 10 + (int)satname[6] - 48;
    }
    else
        satnum = (int)satname[5] - 48 ; //satellite number
    return satnum;
}
int Satellite::MD(char *localSat, char *destSat){

    int localOrbitnum = getOrbitnum(localSat);
    int localSatnum = getSatnum(localSat);

    int destOrbitnum = getOrbitnum(destSat);
    int destSatnum = getSatnum(destSat);

    int xDistance, yDistance;

    xDistance = abs(localOrbitnum - destOrbitnum);
    if(abs(localSatnum - destSatnum) > 5 )
        yDistance = 11 - abs(localSatnum - destSatnum);
    else
        yDistance = abs(localSatnum - destSatnum);

    return xDistance + yDistance;
}
void Satellite::TimeinQue(cMessage *msg, int hop){
    emit(queuingSignal, msg->getArrivalTime() - hop * ISLdelay);
}
double Satellite::fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
