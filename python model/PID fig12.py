import matplotlib.pyplot as plt
import numpy as np

T = 20              #starttemperatuur
beta = 60           #gewenste temperatuur
dt = 0.001          #tijdstap
tijd = 40           #maximale tijd

#constantes voor P, I en D termen
Kp = 20
Ki = 5
Kd = -5

#klaarzetten bepaalde variabelen
I = 0
D = 0
e = beta - T

tijdarray = np.arange(0., tijd, dt)         #array voor tijd maken
temp = np.array([])                         #array voor temperatuur
Ix = np.array([])                           #array voor I term subplot

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
    
    
    T = T + 50*dt * (1 - (T / 150))         #logistieke groei van temperatuur met een limiet van 150 graden   
    if (P+I+D < 0):
        T = T + (P+I+D)*dt   
    
    D = ((T - vorige_T) / dt) * Kd          #D term, moet nu onderaan zodat hij de vorige T en nieuwe T verwerkt
    
    temp = np.append(temp,T)
    Ix = np.append(Ix, I)
    
#dit gedeelte maakt de plot met matplotlib 

plt.subplot(211)
plt.grid(True)                                                 
plt.plot(tijdarray,temp,label='20,5,-5, vasthouden 5 sec')      
plt.ylabel('temperatuur') 
leg = plt.legend()

#naakt een mooie subplot

plt.subplot(212)
plt.grid(True)
plt.plot(tijdarray,Ix, label='I')
plt.xlabel('tijd')
leg = plt.legend()
plt.ylabel('I waarde')
