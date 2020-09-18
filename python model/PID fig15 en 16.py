import matplotlib.pyplot as plt
import numpy as np
import random as rd

T = 20              #starttemperatuur
beta = 60           #gewenste temperatuur
dt = 0.001          #tijdstap
tijd = 40           #tijdaantal

Kp = 20             #constante P
Ki = 5              #constante I
Kd = -2             #constante D
Imax = 50

e = beta - T        #beginfout
I = 0               #gegin I
D = 0               #begin D

tijdarray = np.arange(0., tijd, dt)
temp = np.array([])
Px = np.array([])     
Ix = np.array([])
Dx = np.array([])       

for t in tijdarray:
    if (0<t<5):
        T = 20                           #zorgt ervoor dat T aan de startwaarde gelijk blijft tussen 0 en 5
    if t==20:                            #geeft halverwege een verandering in onze gewenste waarde
        beta = 40
    if t==30:
        beta = 70

    T_vorige = T                         #slaat vorige T op voor afgeleide
    
    e = beta - T                         #bereknt de fout, het verschil tussen de gewenste en huidige waarde
    P = Kp * e                           #P term
    I = I + Ki*e*dt                      #I term
    if (I >= Imax):                      #I mag niet uitkomen boven de maximaal ingestelde waarde
        I = Imax 
    T = T + 50*dt * (1 - (T / 150))      #logistieke groei van temperatuur met een limiet van 150 graden   

    if ((t*100).is_integer()):           #functie die ruis toevoegd, nog delen door 10 omdat het anders enorm onrealistisch wordt.
        T = T + rd.randint(-1,1)/10 
    
    if (P+D+I < 0):                      #deze if omdadt ons systeem alleen kan koelen, niet extra kan opwarmen
        T = T + (P + D)*dt               
        if (I != Imax):                  #de clamping, als hij niet het maximale is dan wordt de integraal er ook bij opgeteld
            T = T + I*dt
    
    D = ((T-T_vorige) / dt) * Kd         #De afgeleide moet na alle T veranderingen worden berekend anders doet hij het niet
    
    temp = np.append(temp,T)                       
    Px = np.append(Px,P*dt)              #voor het plotten van de PID waarden
    Ix = np.append(Ix,I*dt)              #I maal dt omdat het anders onoverzichtelijk wordt in verhouding met de rest
    Dx = np.append(Dx,D*dt)
    
#plot de temperatuur plot (zelf de onderschrift van de lijn aanpassen helaas)

plt.figure(1)
plt.subplot(211)
plt.grid(True)                                                 
plt.plot(tijdarray,temp,label='20,5,-2 met 0<T<5 T=20 en Imax = 50')      
plt.ylabel('temperatuur') 
leg = plt.legend()

#plotten van de PID waarden

plt.subplot(212)
plt.grid(True)
plt.plot(tijdarray,Px, label='P')
#plt.plot(tijdarray,Ix, label='I')
plt.plot(tijdarray,Dx, label='D')
plt.xlabel('tijd')
leg = plt.legend()
plt.ylabel('I waarde')


    



