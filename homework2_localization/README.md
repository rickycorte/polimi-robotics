# Homework 2
Secondo Progetto di robotics anno 2020/2021

## Fatto da:

| nome              | codice persona |
|:------------------|:--------------:|
|Greta Corti        |    10598667    |
|Riccardo Cortelazzo|    10530551    |
|Giorgio Corbetta   |    10570080    |

## Parti da fare:
- [x] Trovare angolo lidar
- [x] Calcolare tf
- [x] Programma (con launch) per calclare la mappa
- [x] Fare la mappa con una bag
- [x] Programma (con launch) per fare la navigazione/localizzazione
- [x] Fare localization con altri bag



## Uso (suppendo che il workspace sia ~/robotics)
- Avviare `roslaunch rb2 amcl.launch` per la localizzazione con amcl (usa una bag diversa da gmap)
- Avviare `roslaunch rb2 gmap.launch` per la mappa da salvare con `rosrun map_server map_saver -f map`
- Per i frame `rosrun tf view_frames`
- Far partire un bag `rosbag play --clock -l 1.bag`

## amcl

Installare tutte le dipendenze
```
sudo apt install ros-melodic-robot-localization
sudo apt install ros-melodic-imu-tools
sudo apt install ros-melodic-rviz-imu-plugin
sudo apt install ros-melodic-stage*
sudo apt install ros-melodic-mapviz*
sudo apt install ros-melodic-gmapping
sudo apt install ros-melodic-amcl
sudo apt install ros-melodic-navigation
```



