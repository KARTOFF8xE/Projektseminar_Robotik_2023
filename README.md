# Projektseminar Robotik 2022/23

## Aufgaben

 - [X] umschreiben: @jkohl
   - [X] wayfinding auf rosbag
   - [X] range_filter
   - [X] `CMakeLists.txt` für wayfinding (Nutzung von bumd_detection_lib und filters_lib)
   - [X] Launchfiles sollen automatisch rosbag2 starten
 - [X] über weitere Filter(-algorithmen) belesen @jkohl
 - [X] über rviz2 Funktionsweise informieren und testen @gmuck
 - [X] Lidar-Algorithmen: @gmuck
    - [X] Bordsteinerkennung
    - [X] Test1: Bordsteinerkennung
    - [X] Fehlerbehebung nach 1. Test
      - [X] Anti-Schlagloch-Threshold soll abhängig von Distanz sein, nicht von Anzahl an Rays
      - [X] auch gekrümmte Straßen werden als Bordsteine erkannt -> soll behoben werden
      - [X] Rechnung überprüfen und neue Rechnung implementieren
      - [X] neue Rechnung validieren durch Visualisieren
      - [X] Roboter ebenso skizziert Visualisieren
      - [X] sind Rechts und Links vertauscht? Wie ist die Drehrichtung des Sensors -> herausfinden -> sind nicht vertauscht
    - [X] Test2: Bordsteinerkennung
      - Erkenntnisse: Schon zuverlässiger (gekrümmte Seiten werden nicht zwangsweiße erkannt, da drive_line-Höhe noch ungünstig bestimmt wird)
        - Ideen: <br>
          &rarr; ggf Neigungssensor in Transformation mit einbeziehen (somit würde man die Drive-Line Neigen? <br>
            &rarr; drive_line als lineare Funktion basteln? &rarr; woher kommt der Neigungswinkel? <br>
            &rarr; Aufgenommenen Sensordaten drehen und somit die Neigung korrigieren? <br>
          &rarr; Unterschiedliche drive_lines für rechte und linke Seite? &rarr; überhaupt Sinnvoll, da Abstand zum Boden Roboterbezogen ist und somit vom Roboter ausgehend gleich bleibt? <br>
          - weitere AlgorithmusIdee: <br>
            - Vektorwinkelbasierter Algorithmus:
              &rarr; erst ganz leichtes Glätten der Linie <br>
              &rarr; anschließendes ausmachen von Bordsteinen anhand von Linienlänge und -winkel (derzeit nur Winkel realisiert).
            - ABER: wie erkennen wir Schlaglöcher?
              &rarr; Durch Vergleich der Höhendifferenz weiterer Werte zum letzten Wert vor starkem Anstieg
          - [X] Vektorwinkelbasierten Algorithmus schreiben
            - [X] Bugs beheben (wenn Hinderniss zu weit nach oben geht, ist es definitiv kein Schlagloch, sondern direkt eine Grenze)
    - [X] Ideen für weiterführung: <br>
        - [X] Wegbreite in Vektor speichern <br>
        - [X] bis zu 1000Pkt via opencv Visualisieren (älteste Pktgruppe wird stets wieder raus geworfen) <br>
        - [X] Filter realisieren um Ausreißer zu erkennen, bzw um Zuverlässigkeit der herausgefundenen Daten zu bestimmen (bspw Bubblefilter) <br>
        - [X] Parametrisieren
    - [X] Test3: Bordsteinerkennung
     - [X] Filter kalibrieren
     - [X] DurchschnittsDistanz ausrechnen um Validität zu bestimmen <br>
     - [X] Inselfilter hinzufügen, um Inseln wegzubekommen -> wenig Aussagekraft, stören eher
    - [X] Test4: Bordsteinerkennung:
      &rarr; ist Mittlerweile eigentlich echt solide <br>
    &rarr; Stand ZwischenStandsvortrag
    - [X] Glättung der Grenzen Realisieren &rarr; überhaupt Sinnvoll? &xrarr; war nicht sinnvoll, feature wurde entfernt
    - [X] Visualisierungspkg schreiben und testen
    - [X] Filter in eigenes Pkg extrahieren um es auch im wayfinding nutzen zu können
 - [X] Algorithmustests:
   - [X] bump_detection
   - [X] range_filter @jkohl
 - [X] Vorfilter & Linienanalyse: @nzumpe
   - [X] Patternmatching für Wegarten testen
   - [X] verallg. Hough-Trafo
        - [X] Metrik für Bewertung der Linien implementieren <br>
            - [X] Konzept für eine Bewertungsmetrik <br>
            - [X] Anpassung Winkelfilter <br>
        - [X] Anpassung der Vorfilter & Verknüpfung mit Metrik
        - [X] Testreihe Metrik
        - [X] Evaluierung der Tests
        - [X] Linienbewertung und Kriterien implementieren
   - [X] Linienanalyse
        - [X] Einbinden der Metrik in Analyse

 - [X] Final branch aufsetzen und befüllen
 - [X] Readmes schreiben

## Setup & Info

### Repository usage

This Repository is meant as an assortment of ros2 (foxy) packages. Therefore it is to be used/cloned as the src folder in your ros workspace.

```
ros2_ws/
├─ build/
│  └─ …
├─ install/
│  └─ …
└─ src/ <- This Repository
   └─ …
```

### rosbag foxy-future setup

```bash
host> sudo docker pull ros:foxy
host> sudo docker run --rm --name ros2foxy -itd -v path/to/rosbags/on/host:path/to/rosbags/on/docker ros:foxy
host> sudo docker attach ros2foxy


docker> sudo apt update
docker> sudo apt install ros-foxy-test-msgs python-pybind11

docker> source /opt/ros/foxy/setup.bash
docker> mkdir -p ~/rosbag_ws/src
docker> cd ~/rosbag_ws/src
docker> git clone -b foxy-future https://github.com/ros2/rosbag2.git
docker> colcon build

docker> source install/setup.bash
```

## Teilnehmername + MatrikelNr
- Johannes Kohl (@jkohl): 64751
- Georg Muck (@gmuck): 63652
- Nico Zumpe (@nzumpe): 62351
