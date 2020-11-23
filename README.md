**ARDUINO CODE**

De code voor de Arduino komt pas rond hoofdstuk 2 en 3.

Python Code

In hoofdstuk 1 van PWS zijn verschillende figuren gebruikt om concepten van een PID-systeem uit te leggen. Deze figuren zijn genummerd in het document en in de code. De code om de figuren te maken is te vinden in de map "python code" en is genummerd op basis van de figuren.

Voor het maken van de figuren in python is het volgende programma gebruikt:

-anaconda navigator: https://www.anaconda.com/products/individual 

Wanneer dit is geïnstalleerd kan je via “anaconda navigator” het programma “Spyder” openen, dit is de python IDE die gebruikt is. Om de grafieken interactief te maken met een hogere resolutie op een apart tabblad kan je eenmalig de volgende instructies volgen (let op: dit is nodig voor meerdere lijnen in 1 grafiek): 

Bovenaan in de navigatiebalk bij Spyder heb je het knopje Tools, klik hier op en ga naar

“preferences” -> “IPython console” -> “graphics” -> “Graphics backend”

Verander hier “Inline” naar “Automatic” en druk op “apply”.

Nu moet je rechtsboven bij de console op de 3 horizontale streepjes drukken en dan op “restart kernel” klikken (of control + . ) Hierna verschijnen de plots op een ander tabblad.


Instructies om het gewenste figuur na te maken:

1: Kijk welk figuur je wilt namaken, en zoek de code op met dezelfde naam (figuur 12 zal dan “PID fig12.py” heten)

2: open de code met Spyder.

3: verander de constanten/variabelen bovenaan de code en verander de label onderaan de code op basis hiervan. De constanten/variabelen die gebruikt zijn staan rondom het figuur in het document. (voor figuur 12 specifiek is het belangrijk dat voor de blauwe lijn, van regel 24 en 25 een comment is gemaakt door er een # voor te zetten)

4: Run de code met het knopje die een groene pijl aangeeft en klik de grafiek niet weg als er meerdere lijnen in het figuur zijn.

5: Verander nu weer de constanten en de label en run de code opnieuw, nu zullen er 2 lijnen verschijnen in het figuur. Herhaal dit totdat alle lijnen zijn nagemaakt.

Let op: Spyder zal ALLEEN meerdere lijnen vertonen als het is ingesteld om in een apart tabblad de grafiek te laten zien. 






**ARDUINO CODE**

Bij de arduino code is het mogelijk om de code te kopiëren en te plakken in de Arduino IDE software, en die vervolgens te uploaden in de robot. Verder moeten de libraries "Accelstepper" en "wire.h" geinstalleerd worden om de codes werkend te krijgen.



