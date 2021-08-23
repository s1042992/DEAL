//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

//
// TODO auto-generated type
//
//import queuenet.Queue;

simple Satellite 
{
	parameters:
		
		@signal[arrival](type="long");
		@signal[delay](type="float");
		@signal[dropmsg](type="long");
		@signal[energy](type="long");
		@signal[queue](type="float");
		@signal[occupy](type="float");
		
		@statistic[delay](title="delay"; source="delay"; record=mean, sum, stats, vector;interpolationmode=none);
		@statistic[hopCount](title="hop count"; source="arrival"; record=mean,vector,stats; interpolationmode=none);
		@statistic[queue](title="Queuing time"; source="queue"; record=mean, sum, vector,stats; interpolationmode=none);
		@statistic[dropPacketTimes](title="drop packet times"; source="dropmsg"; checkSignals=false; record=mean,count,vector,stats; interpolationmode=none);
		@statistic[batteryEnergy](title="Battery energy"; source="energy"; checkSignals=false; record=mean,last,histogram,vector,stats; interpolationmode=none);
		@statistic[occupy](title="Empty-Full Ratio of Queue"; source="occupy"; record=mean,sum,vector,stats; interpolationmode=none);
		@display("i=device/satellite");
        //@display("i=block/routing");
		
		
		
		
    gates:
        inout upper;
		inout lower;
		inout left;
		inout right;
		
}


network SatNet
{
	types:
		channel Channel extends ned.DatarateChannel{
			delay = 32.97us;
			datarate = 15.0Mbps;
			ber = 1e-10;
		}
		
		//inter-plane ISL:
		channel interISL extends ned.DatarateChannel{
			//about 3297.35 km
			//equal 0.00003297s = 0.03297ms = 32.97us 
			delay = 32.97us;
			datarate = 15.0Mbps;
			ber = 1e-10;
		}
		channel ainterISL extends ned.DatarateChannel{
			
			//equal 0.00134585903s = 1.346ms = 1345.85903us 
			delay = 1345.85903us;
			datarate = 15.0Mbps;
			ber = 1e-10;
		}
		
		channel binterISL extends ned.DatarateChannel{
			
			//equal 0.001166s = 1.166ms = 1166us 
			delay = 1166us;
			datarate = 15.0Mbps;
			ber = 1e-10;
		}
		
		channel cinterISL extends ned.DatarateChannel{
			
			//equal 0.00067292951s = 0.67293ms = 673us 
			delay = 673us;
			datarate = 15.0Mbps;
			ber = 1e-10;
		}
		
		
		
		//intra-plane ISL:
		channel intraISL extends ned.DatarateChannel{
			delay = 32.97us;
			datarate = 15.0Mbps;
			ber = 1e-10;
		}
	submodules:
        
		sat1_1:Satellite{ @display("p=150,50");}
		sat1_2:Satellite{ @display("p=150,130");}
		sat1_3:Satellite{ @display("p=150,210");}
		sat1_4:Satellite{ @display("p=150,290");}
		sat1_5:Satellite{ @display("p=150,370");}
        sat1_6:Satellite{ @display("p=150,450");}
		sat1_7:Satellite{ @display("p=150,530");}
		sat1_8:Satellite{ @display("p=150,610");}
		sat1_9:Satellite{ @display("p=150,690");}
		sat1_10:Satellite{ @display("p=150,770");}
		sat1_11:Satellite{ @display("p=100,820");}
		
		sat2_1:Satellite{ @display("p=250,50");}
		sat2_2:Satellite{ @display("p=250,130");}
		sat2_3:Satellite{ @display("p=250,210");}
		sat2_4:Satellite{ @display("p=250,290");}
		sat2_5:Satellite{ @display("p=250,370");}
        sat2_6:Satellite{ @display("p=250,450");}
		sat2_7:Satellite{ @display("p=250,530");}
		sat2_8:Satellite{ @display("p=250,610");}
		sat2_9:Satellite{ @display("p=250,690");}
		sat2_10:Satellite{ @display("p=250,770");}
		sat2_11:Satellite{ @display("p=200,820");}
		
		sat3_1:Satellite{ @display("p=350,50");}
		sat3_2:Satellite{ @display("p=350,130");}
		sat3_3:Satellite{ @display("p=350,210");}
		sat3_4:Satellite{ @display("p=350,290");}
		sat3_5:Satellite{ @display("p=350,370");}
        sat3_6:Satellite{ @display("p=350,450");}
		sat3_7:Satellite{ @display("p=350,530");}
		sat3_8:Satellite{ @display("p=350,610");}
		sat3_9:Satellite{ @display("p=350,690");}
		sat3_10:Satellite{ @display("p=350,770");}
		sat3_11:Satellite{ @display("p=300,820");}
		
		sat4_1:Satellite{ @display("p=450,50");}
		sat4_2:Satellite{ @display("p=450,130");}
		sat4_3:Satellite{ @display("p=450,210");}
		sat4_4:Satellite{ @display("p=450,290");}
		sat4_5:Satellite{ @display("p=450,370");}
        sat4_6:Satellite{ @display("p=450,450");}
		sat4_7:Satellite{ @display("p=450,530");}
		sat4_8:Satellite{ @display("p=450,610");}
		sat4_9:Satellite{ @display("p=450,690");}
		sat4_10:Satellite{ @display("p=450,770");}
		sat4_11:Satellite{ @display("p=400,820");}
		
		sat5_1:Satellite{ @display("p=550,50");}
		sat5_2:Satellite{ @display("p=550,130");}
		sat5_3:Satellite{ @display("p=550,210");}
		sat5_4:Satellite{ @display("p=550,290");}
		sat5_5:Satellite{ @display("p=550,370");}
        sat5_6:Satellite{ @display("p=550,450");}
		sat5_7:Satellite{ @display("p=550,530");}
		sat5_8:Satellite{ @display("p=550,610");}
		sat5_9:Satellite{ @display("p=550,690");}
		sat5_10:Satellite{ @display("p=550,770");}
		sat5_11:Satellite{ @display("p=500,820");}
		
		sat6_1:Satellite{ @display("p=650,50");}
		sat6_2:Satellite{ @display("p=650,130");}
		sat6_3:Satellite{ @display("p=650,210");}
		sat6_4:Satellite{ @display("p=650,290");}
		sat6_5:Satellite{ @display("p=650,370");}
        sat6_6:Satellite{ @display("p=650,450");}
		sat6_7:Satellite{ @display("p=650,530");}
		sat6_8:Satellite{ @display("p=650,610");}
		sat6_9:Satellite{ @display("p=650,690");}
		sat6_10:Satellite{ @display("p=650,770");}
		sat6_11:Satellite{ @display("p=600,820");}
		
    connections allowunconnected:
		//inter-plane1
        sat1_1.lower <--> cinterISL <--> sat1_2.upper;
        sat1_2.lower <--> binterISL <--> sat1_3.upper;
        sat1_3.lower <--> ainterISL <--> sat1_4.upper;
        sat1_4.lower <--> ainterISL <--> sat1_5.upper;
		sat1_5.lower <--> binterISL <--> sat1_6.upper;
		sat1_6.lower <--> cinterISL <--> sat1_7.upper;
		sat1_7.lower <--> cinterISL <--> sat1_8.upper;
		sat1_8.lower <--> binterISL <--> sat1_9.upper;
		sat1_9.lower <--> ainterISL <--> sat1_10.upper;
		sat1_10.lower <--> binterISL <--> sat1_11.upper;
		sat1_11.lower <--> cinterISL <--> sat1_1.upper;
		
		//inter-plane2
		sat2_1.lower <--> cinterISL <--> sat2_2.upper;
        sat2_2.lower <--> binterISL <--> sat2_3.upper;
        sat2_3.lower <--> ainterISL <--> sat2_4.upper;
        sat2_4.lower <--> ainterISL <--> sat2_5.upper;
		sat2_5.lower <--> binterISL <--> sat2_6.upper;
		sat2_6.lower <--> cinterISL <--> sat2_7.upper;
		sat2_7.lower <--> cinterISL <--> sat2_8.upper;
		sat2_8.lower <--> binterISL <--> sat2_9.upper;
		sat2_9.lower <--> ainterISL <--> sat2_10.upper;
		sat2_10.lower <--> binterISL <--> sat2_11.upper;
		sat2_11.lower <--> cinterISL <--> sat2_1.upper;
		
		//inter-plane3
		sat3_1.lower <--> cinterISL <--> sat3_2.upper;
        sat3_2.lower <--> binterISL <--> sat3_3.upper;
        sat3_3.lower <--> ainterISL <--> sat3_4.upper;
        sat3_4.lower <--> ainterISL <--> sat3_5.upper;
		sat3_5.lower <--> binterISL <--> sat3_6.upper;
		sat3_6.lower <--> cinterISL <--> sat3_7.upper;
		sat3_7.lower <--> cinterISL <--> sat3_8.upper;
		sat3_8.lower <--> binterISL <--> sat3_9.upper;
		sat3_9.lower <--> ainterISL <--> sat3_10.upper;
		sat3_10.lower <--> binterISL <--> sat3_11.upper;
		sat3_11.lower <--> cinterISL <--> sat3_1.upper;
		
		//inter-plane4
		sat4_1.lower <--> cinterISL <--> sat4_2.upper;
        sat4_2.lower <--> binterISL <--> sat4_3.upper;
        sat4_3.lower <--> ainterISL <--> sat4_4.upper;
        sat4_4.lower <--> ainterISL <--> sat4_5.upper;
		sat4_5.lower <--> binterISL <--> sat4_6.upper;
		sat4_6.lower <--> cinterISL <--> sat4_7.upper;
		sat4_7.lower <--> cinterISL <--> sat4_8.upper;
		sat4_8.lower <--> binterISL <--> sat4_9.upper;
		sat4_9.lower <--> ainterISL <--> sat4_10.upper;
		sat4_10.lower <--> binterISL <--> sat4_11.upper;
		sat4_11.lower <--> cinterISL <--> sat4_1.upper;
		
		//inter-plane5
		sat5_1.lower <--> cinterISL <--> sat5_2.upper;
        sat5_2.lower <--> binterISL <--> sat5_3.upper;
        sat5_3.lower <--> ainterISL <--> sat5_4.upper;
        sat5_4.lower <--> ainterISL <--> sat5_5.upper;
		sat5_5.lower <--> binterISL <--> sat5_6.upper;
		sat5_6.lower <--> cinterISL <--> sat5_7.upper;
		sat5_7.lower <--> cinterISL <--> sat5_8.upper;
		sat5_8.lower <--> binterISL <--> sat5_9.upper;
		sat5_9.lower <--> ainterISL <--> sat5_10.upper;
		sat5_10.lower <--> binterISL <--> sat5_11.upper;
		sat5_11.lower <--> cinterISL <--> sat5_1.upper;
		
		//inter-plane6
		sat6_1.lower <--> cinterISL <--> sat6_2.upper;
        sat6_2.lower <--> binterISL <--> sat6_3.upper;
        sat6_3.lower <--> ainterISL <--> sat6_4.upper;
        sat6_4.lower <--> ainterISL <--> sat6_5.upper;
		sat6_5.lower <--> binterISL <--> sat6_6.upper;
		sat6_6.lower <--> cinterISL <--> sat6_7.upper;
		sat6_7.lower <--> cinterISL <--> sat6_8.upper;
		sat6_8.lower <--> binterISL <--> sat6_9.upper;
		sat6_9.lower <--> ainterISL <--> sat6_10.upper;
		sat6_10.lower <--> binterISL <--> sat6_11.upper;
		sat6_11.lower <--> cinterISL <--> sat6_1.upper;
		
		//-------------------------------------------
		
		//intra-plane 1,2
		sat1_1.right <--> intraISL <--> sat2_1.left;
        sat1_2.right <--> intraISL <--> sat2_2.left;
        sat1_3.right <--> intraISL <--> sat2_3.left;
        sat1_4.right <--> intraISL <--> sat2_4.left;
		sat1_5.right <--> intraISL <--> sat2_5.left;
		sat1_6.right <--> intraISL <--> sat2_6.left;
		sat1_7.right <--> intraISL <--> sat2_7.left;
		sat1_8.right <--> intraISL <--> sat2_8.left;
		sat1_9.right <--> intraISL <--> sat2_9.left;
		sat1_10.right <--> intraISL <--> sat2_10.left;
		sat1_11.right <--> intraISL <--> sat2_11.left;
		
		//intra-plane 2,3
		sat2_1.right <--> intraISL <--> sat3_1.left;
        sat2_2.right <--> intraISL <--> sat3_2.left;
        sat2_3.right <--> intraISL <--> sat3_3.left;
        sat2_4.right <--> intraISL <--> sat3_4.left;
		sat2_5.right <--> intraISL <--> sat3_5.left;
		sat2_6.right <--> intraISL <--> sat3_6.left;
		sat2_7.right <--> intraISL <--> sat3_7.left;
		sat2_8.right <--> intraISL <--> sat3_8.left;
		sat2_9.right <--> intraISL <--> sat3_9.left;
		sat2_10.right <--> intraISL <--> sat3_10.left;
		sat2_11.right <--> intraISL <--> sat3_11.left;
		
		//intra-plane 3,4
		sat3_1.right <--> intraISL <--> sat4_1.left;
        sat3_2.right <--> intraISL <--> sat4_2.left;
        sat3_3.right <--> intraISL <--> sat4_3.left;
        sat3_4.right <--> intraISL <--> sat4_4.left;
		sat3_5.right <--> intraISL <--> sat4_5.left;
		sat3_6.right <--> intraISL <--> sat4_6.left;
		sat3_7.right <--> intraISL <--> sat4_7.left;
		sat3_8.right <--> intraISL <--> sat4_8.left;
		sat3_9.right <--> intraISL <--> sat4_9.left;
		sat3_10.right <--> intraISL <--> sat4_10.left;
		sat3_11.right <--> intraISL <--> sat4_11.left;
		
		//intra-plane 4,5
		sat4_1.right <--> intraISL <--> sat5_1.left;
        sat4_2.right <--> intraISL <--> sat5_2.left;
        sat4_3.right <--> intraISL <--> sat5_3.left;
        sat4_4.right <--> intraISL <--> sat5_4.left;
		sat4_5.right <--> intraISL <--> sat5_5.left;
		sat4_6.right <--> intraISL <--> sat5_6.left;
		sat4_7.right <--> intraISL <--> sat5_7.left;
		sat4_8.right <--> intraISL <--> sat5_8.left;
		sat4_9.right <--> intraISL <--> sat5_9.left;
		sat4_10.right <--> intraISL <--> sat5_10.left;
		sat4_11.right <--> intraISL <--> sat5_11.left;
		
		//intra-plane 5,6
		sat5_1.right <--> intraISL <--> sat6_1.left;
        sat5_2.right <--> intraISL <--> sat6_2.left;
        sat5_3.right <--> intraISL <--> sat6_3.left;
        sat5_4.right <--> intraISL <--> sat6_4.left;
		sat5_5.right <--> intraISL <--> sat6_5.left;
		sat5_6.right <--> intraISL <--> sat6_6.left;
		sat5_7.right <--> intraISL <--> sat6_7.left;
		sat5_8.right <--> intraISL <--> sat6_8.left;
		sat5_9.right <--> intraISL <--> sat6_9.left;
		sat5_10.right <--> intraISL <--> sat6_10.left;
		sat5_11.right <--> intraISL <--> sat6_11.left;
}
