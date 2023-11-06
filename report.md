# Logica dataAssociation
Per l'associazione dei traklet sono state implementate entrambe le metriche (distanza euclidea e malhanobis) per valutare la vicinanza al cluster individuato. Per utilizzare l'una o l'altra basta settare la variabile DISTANCE_TYPE a 0 per l'euclidea e qualsiasi altro valore per
quella di mhalanobis.

# Funzionalità aggiuntive

## Dstanza maggiore
È stata implementato il calcolo del truklet che ha percorso una distanza maggiore. Quest'ultimo è evidenziato colorando il proprio id in verde. Per realizzarlo è stato aggiunta una variabile per il salvataggio dei metri percorsi che viene agigornata 
nell'update function, quindi ogni volta che ricevo un cluster e tramite la distanza euclidea vado a sommare la distanza percorsa.

## Area
È stata anche realizzare la funzionalità per cui se un traklet entra in un area definita da un cetro e un raggio allora aggiorno
il contatore che identifica il numero di traklet che sono entrati in quell'area. Per farlo varifico che la distanza tra traklet e centro
dell'area sia minore o uguale del raggio definito e salvo l'id del traklet in un vettore per meorizzare chi è entrato.

# Miglioramenti 
Per ottenere un miglior tracciamento il lavoro maggiore deve essere eseguito a livello di clustering, anche perchè come è mostrato nell'esempio proposto quello che avviene è che molti cluster non vengano rilevati il che porta al tucker a non poter associarlo. Poi ci sono molti casi come i cluster molto vicini, in quel caso abbiamo un problema difficilmente risolvibile anche perchè i 2 cluster vengono identificati come uno solo.