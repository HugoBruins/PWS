import matplotlib.pyplot as plt
import numpy as np

T = 20              #starttemperatuur
beta = 60           #gewenste temperatuur
dt = 0.001          #tijdstap
tijd = 25           #maximale tijd

Kp = 1
Ki = 1

I = 0

tijdarray = np.arange(0., tijd, dt)         #array voor tijd maken
temp = np.array([])                         #array voor temperatuur 

for t in tijdarray:
    e = beta - T                            #berekent de fout, het verschil tussen de gewenste en huidige waarde
    P = Kp * e                              #D term
    I = I + Ki*e*dt                         #I term
    
    T = T + 50*dt * (1 - (T / 150))         #logistieke groei van temperatuur met een limiet van 150 graden   
    if (P+I < 0):
        T = T + (I+P)*dt   
        
    temp = np.append(temp,T)
    
#dit gedeelte maakt de plot met matplotlib 

plt.figure(4)
plt.grid(True)                                                 
plt.plot(tijdarray,temp,label='temperatuur met Kp = 1, Ki = 1')      
plt.ylabel('temperatuur') 
plt.xlabel('tijd')
leg = plt.legend()