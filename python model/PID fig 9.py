import matplotlib.pyplot as plt
import numpy as np

T = 20              #starttemperatuur
beta = 60           #gewenste temperatuur
dt = 0.001          #tijdstap
tijd = 40           #maximale tijd

#constantes voor P, I en D termen
Kp = 20
Ki = 5
Kd = 5

#klaarzetten bepaalde variabelen
I = 0
e = beta - T

tijdarray = np.arange(0., tijd, dt)         #array voor tijd maken
temp = np.array([])                         #array voor temperatuur 

for t in tijdarray:
    if (t==20):                             #veranderd de gewenste waarde
        beta = 40
    if (t==30):
        beta = 70
    
    vorige_e = e
    e = beta - T                            #berekent de fout, het verschil tussen de gewenste en huidige waarde
    P = Kp * e                              #P term
    I = I + Ki*e*dt                         #I term
    D = ((e - vorige_e) / dt) * Kd          #D term
    
    T = T + 50*dt * (1 - (T / 150))         #logistieke groei van temperatuur met een limiet van 150 graden   
    if (P+I+D < 0):
        T = T + (P+I+D)*dt   
        
    temp = np.append(temp,T)
    
#dit gedeelte maakt de plot met matplotlib 

plt.figure(9)
plt.grid(True)                                                 
plt.plot(tijdarray,temp,label='temperatuur met Kp = 20, Ki = 5, Kd = 5')      
plt.ylabel('temperatuur') 
plt.xlabel('tijd')
leg = plt.legend()
