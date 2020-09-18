import matplotlib.pyplot as plt
import numpy as np

T = 20              #starttemperatuur
dt = 0.001          #tijdstap
tijd = 25           #maximale tijd


tijdarray = np.arange(0., tijd, dt)         #array voor tijd maken
temp = np.array([])                         #array voor temperatuur 

for t in tijdarray:
    T = T + 50*dt * (1 - (T / 150))         #logistieke groei van temperatuur met een limiet van 150 graden   
    temp = np.append(temp,T)   

    
#dit gedeelte maakt de plot met matplotlib 

plt.figure(1)
plt.grid(True)                                                 
plt.plot(tijdarray,temp,label='temperatuur zonder controle')      
plt.ylabel('temperatuur') 
plt.xlabel('tijd')
leg = plt.legend()



    



