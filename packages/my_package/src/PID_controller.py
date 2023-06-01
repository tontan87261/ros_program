#!/usr/bin/env python3
import rospy

#-------------------------------------- PID KONTROLLERI ARVUTUSED ----------------------------------------------#
def PID(position, delta_t, prev_e):
    
    In_al = 0
    PID = 0
    #Joone järgimine
    Kp = 0.051#float(rospy.get_param("/p"))#0.04
    Ki = 0.02#float(rospy.get_param("/i"))#0.02     
    Kd = 0.0019#float(rospy.get_param("/d"))#2   #Vähendab roboti ujumist 
    delta = 1/20
#töötas:
#P: 0.045
#I:0.05
#D:0.06
    
    if delta_t == 0:
        print("Waiting for delta_t")
    else:
        #print("Posititon: ", position)
        error = position
            
        #In_al = In_al + (delta * error ) #in_al = integral VANA INTEGRAAL
        In_al = In_al + (error + prev_e)*delta/2
        In_al = max(min(In_al, 1.8), -1.8)
        if error == 0:
            In_al = 0
        err_der = error - prev_e / delta
        #err_der = max(min(err_der, 2), -2)
    
        Kpe = Kp * error
        Kie = Ki * In_al
        Kde = Kd * err_der

        PID = Kpe + Kie + Kde
        '''
        print("P ", Kpe)
        print("I ", Kie)
        print("D ", Kde)
        print("Inte: ", In_al)
        print("", err_der)
        print("error: ", error)
        print("last error: ", prev_e)
        print("")
        '''

    #print("PID FAILIS: ", self.PID)
    
    return PID, error