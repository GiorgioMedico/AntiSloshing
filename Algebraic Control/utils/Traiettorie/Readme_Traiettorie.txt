trajectory:
Contiene le traiettorie del contenitore del liquido, partendo da posizione e orientamento zero.

--------------
joint:
Contiene alcune traiettorie di giunto corrispondenti a quelle del contenitore.

--------------
joint_back:
Contiene le traiettorie che permettono di tornare alla configurazione di partenza per l'esecuzione di determinate traiettorie:
1. Quelle con "BACK_" permettono di tornare alla configurazione iniziale dopo l'esecuzione della traiettoria corrispondente
2. Quelle con X_to_Y permettono di passare dalla configurazione iniziale del tipo di traiettoria X a quello del tipo di traiettoria Y

