# we want to map 1-15 to 0 to 160:
# 14 intervals

import math

interval=float(15.0/160.0)
add=1

#print interval
for x in range(160):
    print str(int(math.floor(add+(x*interval))))+",", 

# now test map of 160 to 512

test=512-160
interval=float(16.0/test)
add=15

#print interval
for x in range(test):
    print str(int(math.floor(add+(x*interval))))+",", 

# now test map of 512 to 868

test=868-512
interval=float(16.0/test)
add=31

#print interval
for x in range(test):
    print str(int(math.floor(add+(x*interval))))+",", 

    
# now map 869 to 1022 to  62     

test=1022-869
interval=float(16.0/test)
add=47

#print interval
for x in range(test):
    print str(int(math.floor(add+(x*interval))))+",", 
