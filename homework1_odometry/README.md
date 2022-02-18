# Homework 1
Progetto di robotics anno 2020/2021

## Fatto da:

| nome              | codice persona |
|:------------------|:--------------:|
|Greta Corti        |    10598667    |
|Riccardo Cortelazzo|    10530551    |
|Giorgio Corbetta   |    10570080    |

## Parti da fare:
- [x] Filtrare messaggio per sincronizzare la velocità dei motori
- [x] Stimare la velocità lineare e la velocità angolare
- [x] calcolare l'odometria
- [ ] -opt- Usare la posizione reale per calibrare l'odometria


## Uso (suppendo che il workspace sia ~/robotics)
- Assicurarsi di aver copiato la cartella `robotics_hw1` in `rotics/src`
- Assicurarsi di avere `source /home/<usr>/robotis/devel/setup.bash` in `.bashrc`
- Avviare `catkin_make && roslaunch rb1 up.launch` per avviare tutti i nodi

Per il config dinamico `rosrun rqt_reconfigure rqt_reconfigure`
 `rosbag play --clock -l bag1.bag`

