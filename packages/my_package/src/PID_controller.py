#!/usr/bin/env python3

#-------------------------------------- PID KONTROLLERI ARVUTUSED ----------------------------------------------#
def PID(position, delta_t, prev_e):
    prev_e = 0
    In_al = 0
    PID = 0
    #Joone järgimine
    Kp = 0.048
    Ki = 0.019 
    Kd = 0.0017  #Vähendab roboti ujumist 
    delta = 1/20

#töötas:
#P: 0.045
#I:0.05
#D:0.06

    #Mida vaja:
    #delta_t, position

    #P
    
    
    if delta_t == 0:
        print("Waiting for delta_t")
    else:
        #print("Posititon: ", position)
        error = position
        #print("Error: ", error)
        #I - integral
            
        #In_al = In_al + (delta * error ) #in_al = integral
        In_al = In_al + (error+prev_e)*delta/2
        #print("integral: ", self.In_al)
        #anti-windup - väldib integraalvea liigset suurenemist
        In_al = max(min(In_al, 1.8), -1.8)
        if error == 0:
            In_al = 0
        #print("integral + anti-windup: ", self.In_al)
        #D - derivative
        #print (delta_t)
        err_der = error - prev_e / delta
        err_der = max(min(err_der, 2), -2)
        
        #print("P ", error)
        #print("I ", In_al)
        
        #print("D ", err_der)
        
            
        Kpe = Kp * error
        Kie = Ki * In_al
        Kde = Kd * err_der
        #print("Kpe: ", Kpe)
        #print(round(Kpe, 2), round(Kie, 2), round(Kde, 2))
        PID = Kpe + Kie + Kde
        prev_e = error
    
    #print("PID FAILIS: ", self.PID)
    
    return PID, error