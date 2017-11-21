from math import *

def _nearest_equivalent_angle(desired, current):
    previous_rev = floor(current/(2*pi))
    next_rev = ceil(current/(2*pi))
    
    if fabs(current - previous_rev*2*pi) < fabs(current - next_rev*2*pi):
        current_rev = previous_rev
    else:
        current_rev = next_rev
     
    # Determine closestAngle
    low_val = (current_rev - 1)*2*pi + desired
    med_val = current_rev*2*pi + desired
    high_val = (current_rev + 1)*2*pi + desired
    
    if (fabs(current - low_val) <= fabs(current - med_val) and fabs(current - low_val) <= fabs(current - high_val)):
        return low_val
    elif (fabs(current - med_val) <= fabs(current - low_val) and fabs(current - med_val) <= fabs(current - high_val)):
        return med_val
    else:
        return high_val 

desired = input("desired = ")
current = input("current = ")
print "Result =", _nearest_equivalent_angle(desired, current)

