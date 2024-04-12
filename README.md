**Token Passing**

I file principali che compongono il progetto sono i seguenti:

-config.json: la sua funzione principale è l'indicare quale ambiente(mappa) verrà usato durante la simulazione. Di base è impostato per considerare una mappa nella cartella "Environments".

-Le mappe nella cartella "Environments": sono file di tipo .yaml in cui sono indicati gli agenti con la loro prosizione iniziale, i vari ostacoli, le posizioni possibili dove generare i pickup(start) ed i delivery(goal) ed i non-task endpoint (è buona norma che le posizioni iniziali degli agenti siano un sottoinsieme dei non-task endpoint). 

Nella cartella Simulation:
-simulation.py: la sua funzione principale è di cambiare la posizione degli agenti, ad ogni time step, in modo che seguano i percorsi che hanno pianificato.

-TP.py: implementa l'algoritmo token passing

**Per eseguire una run**

Usare il file demo.py che una volta eseguito legge il file di configurazione ed esegue la simulazione. Alla fine viene generato un video in cui si possono vedere i vari spostamenti degli agenti.

Per PyCharm: se alla fine dell'esecuzione dovesse esserci un problema nella riproduzione del video andare in settings-> Tools -> Python Plots -> e rimuovere la spunta dalla voce "Show plots in tool window"


