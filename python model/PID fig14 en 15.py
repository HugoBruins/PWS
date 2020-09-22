import matplotlib.pyplot as plt
import numpy as np
import random as rd

T = 20              #starttemperatuur
beta = 60           #gewenste temperatuur
dt = 0.001          #tijdstap
tijd = 40           #maximale tijd

#constantes voor P, I en D termen en Imax
Kp = 20
Ki = 5
Kd = -2
Imax = 50

#klaarzetten bepaalde variabelen
I = 0
D = 0
e = beta - T

tijdarray = np.arange(0., tijd, dt)         #array voor tijd maken
temp = np.array([])                         #array voor temperatuur
Px = np.array([])                           #array voor I term subplot
Dx = np.array([])                           #array voor D term subplot

for t in tijdarray:
    if (0<t<5):                             #voor het vasthouden, wil je dit niet hebben maak hier een comment van
        T=20
    if (t==20):                             #veranderd de gewenste waarde
        beta = 40
    if (t==30):
        beta = 70
    
    vorige_T = T                            #dit is voor de D term
    e = beta - T                            #berekent de fout, het verschil tussen de gewenste en huidige waarde
    P = Kp * e                              #P term
    I = I + Ki*e*dt                         #I term
    if (I >= Imax):                         #I mag niet uitkomen boven de maximaal ingestelde waarde
        I = Imax 
    
    T = T + 50*dt * (1 - (T / 150))         #logistieke groei van temperatuur met een limiet van 150 graden 
    
    if ((t*100).is_integer()):              #functie voor ruis, gemaakt voor vaste tijdstippen zodat hij met dt schaalt
        T = T + rd.randint(-1,1) / rd.randint(1,12) 
    
    if (P+I+D < 0):
        T = T + (P+D)*dt   
        if (I != Imax):                     #de clamping, als hij niet maximaal is dan wordt de integraal er ook bij opgeteld
            T = T + I*dt
    
    D = ((T - vorige_T) / dt) * Kd          #D term, moet nu onderaan zodat hij de vorige T en nieuwe T verwerkt
    
    temp = np.append(temp,T)
    Px = np.append(Px, P*dt)
    Dx = np.append(Dx, D*dt)
    
#dit gedeelte maakt de plot met matplotlib 

plt.subplot(211)
plt.grid(True)                                                 
plt.plot(tijdarray,temp,label='20,5,-2, Imax = 50')      
plt.ylabel('temperatuur (℃)') 
leg = plt.legend()

#naakt een mooie subplot

plt.subplot(212)
plt.grid(True)
plt.plot(tijdarray,Px, label='P')
plt.plot(tijdarray,Dx, label='D')
plt.xlabel('tijd (sec)')
leg = plt.legend()
plt.ylabel('P en D waarde')
