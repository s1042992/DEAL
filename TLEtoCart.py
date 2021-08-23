from skyfield.api import EarthSatellite
from skyfield.api import load, wgs84

ts = load.timescale()
#TLE
line1 = '1 41917U 17003A   21232.51016335  .00000084  00000-0  22971-4 0  9994'
line2 = '2 41917  86.3946 104.7364 0001879  78.5544 281.5863 14.34215391240702'
satellite = EarthSatellite(line1, line2, 'IRIDIUM', ts)
print(satellite)
#time format: Year, Month, Day, Hour, Minute, Second
t = ts.utc(2021, 8, 2, 3, 59, 27)

geocentric = satellite.at(t)
print(geocentric.position.km)

#convert cartesian to wgs84
subpoint = wgs84.subpoint(geocentric)
print('Latitude:', subpoint.latitude)
print('Longitude:', subpoint.longitude)
print('Height: {:.1f} km'.format(subpoint.elevation.km))
