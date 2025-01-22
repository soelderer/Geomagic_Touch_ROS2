# Geomagic_Touch_ROS2

# Installation von ROS2
Das Robot Operating System (ROS) ist eine Open-Source-Plattform für die Entwicklung von Robotikanwendungen, die aus einer Sammlung von Software-Bibliotheken, Werkzeugen und Treibern besteht. Es gibt verschiedene Versionen. Wegen längerem Support empfiehlt es sich derzeit "Humble Hawksbill" zu installieren.

ROS2 gibt es für verschiedene Betriebssysteme. Für Ubuntu folgt man dieser Anleitung:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
Wichtig ist, die passende Ubuntu-Version zu haben (hier 22.04).

```
$ sudo apt install ros-humble-desktop
```

```
$ sudo apt install ros-dev-tools
```


# Installation der Gerätetreiber
Nun müssen die Gerätetreiber des Herstellers installiert werden. Dazu lädt man sich auf der folgenden Webseite (1) den OpenHaptics Installer und (2) den Touch Device Driver herunter:
https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US.

### (1) OpenHaptics Installer
Man entpackt das heruntergeladene tar.gz-Archiv und wechselt in den Ordner:
```
~/downloads $ tar xf openhaptics_3.4-0-developer-edition-amd64.tar.gz
~/downloads $ cd openhaptics_3.4-0-developer-edition-amd64
```

Ggf. müssen vor der Installation noch einige Abhängigkeiten installiert werden:
```
~/downloads/openhaptics_3.4-0-developer-edition-amd64 $ sudo apt update
~/downloads/openhaptics_3.4-0-developer-edition-amd64 $ sudo apt install libncurses5 libncurses5-dev freeglut3-dev build-essential
```

Nun führt man das Installationsskript aus (benötigt Root-Rechte) und folgt den Anweisungen:
```
~/downloads/openhaptics_3.4-0-developer-edition-amd64 $ sudo ./install
```

Vor der Verwendung muss man neustarten.

### (2) Touch Device Driver
Man entpackt das heruntergeladene tar.gz-Archiv und wechselt in den Ordner:
```
~/downloads $ tar xf TouchDriver_2023_11_15.tgz
~/downloads $ cd TouchDriver_2023_11_15
```

Nun führt man das Installationsskript aus (benötigt Root-Rechte) und folgt ggf. den Anweisungen:
```
~/downloads/TouchDriver_2023_11_15 $ sudo ./install_haptic_driver
```

Falls man eine Fehlermeldung bekommt vonwegen "unknown: !=", kann man folgendes probieren:
```
~/downloads/TouchDriver_2023_11_15 $ sudo bash install_haptic_driver
```

# Installation des ROS2-Treibers
Der Sourcecode des ROS2-Treibers befindet sich in diesem Repository. Man lädt
es z.B. über die Github-Webseite runter ("<> Code" - "Download zip").

Nach dem Entpacken ...
```
~/downloads $ unzip Geomagic_Touch_ROS2-main
```

...wechselt man in den Ordner:
```
~/downloads $ cd Geomagic_Touch_ROS2
```

Um ROS2 nach der Installation zu nutzen, muss die ROS2-Umgebung initialisiert werden. Dies geschieht durch das Laden der entsprechenden Umgebungsvariablen mit dem folgendem Befehl. Das ist immer nötig, wenn ein neues Terminal geöffnet wird.

```
~/downloads/Geomagic_Touch_ROS2 $ source /opt/ros/humble/setup.bash
```

Jetzt kann der ROS2-Treiber kompiliert werden:
```
~/downloads/Geomagic_Touch_ROS2 $ colcon build --symlink-install --event-handlers console_cohesion+
```

Nach einer Weile sollte das fehlerfrei durchgelaufen sein. Man kann den ROS2-Treiber jetzt installieren:
```
~/downloads/Geomagic_Touch_ROS2 $ source install/setup.bash
```

Zu guter Letzt empfiehlt es sich, die folgenden Zeilen ans Ende der Datei `~/.bashrc` hinzuzufügen, damit die Umgebungen automatisch bereitstehen und man sich das händische Sourcen sparen kann:

```
# Source ROS 2 Humble setup
source /opt/ros/humble/setup.bash
# Source Geogamic Touch setup
source ~/downloads/Geomagic_Touch_ROS2/install/setup.bash`
```
# Verwendung
Wenn alles richtig installiert wurde, kann man den ROS2-Treiber wie folgt verwenden.
Zuerst lädt man den ROS2-Treiber. Das funktioniert nur dann fehlerfrei, wenn das Haptic Device per USB verbunden ist.
```
$ ros2 launch omni_common omni_state.launch.py
```

Wenn alles gut geht, sollte man einen ähnlichen Output sehen:
```
[omni_state-1] [INFO] [1734536088.361789230] [omni_haptic_node]: Found Touch.
[omni_state-1] [INFO] [1734536088.367772981] [omni_haptic_node]: HD_CALIBRATION_INKWELL..
[omni_state-1] [INFO] [1734536088.389827966] [omni_haptic_node]: Publishing button events on: phantom/button
[omni_state-1] [INFO] [1734536088.390549662] [omni_haptic_node]: Publishing omni state on: phantom/state
[omni_state-1] [INFO] [1734536088.391267418] [omni_haptic_node]: listening to: phantom/force_feedback for haptic info
[omni_state-1] [INFO] [1734536088.391902368] [omni_haptic_node]: Publishing pose on: phantom/pose
[omni_state-1] [INFO] [1734536088.393056213] [omni_haptic_node]: Publishing joint state on: phantom/joint_states
[omni_state-1] [INFO] [1734536088.393097152] [omni_haptic_node]: PHaNTOM position given in [mm], ratio [1.0]
[omni_state-1] [INFO] [1734536088.393113172] [omni_haptic_node]: Publishing PHaNTOM state at [1000] Hz
[omni_state-1] $GTDD_HOME not set. Using Default value : /home/paul/.3dsystems
[omni_state-1] : Success
```

Nun kann man den ROS2-Treiber benutzen, um mit dem Gerät zu kommunizieren, z.B. die Position auszulesen.
