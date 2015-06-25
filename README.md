# 3DReconstructie

<b>PCL installeren:</b>

<b>Windows</b>:

De pre-compiled binaries van PCL 1.6 zijn onvoldoende, aangezien die NDT nog niet bevatten.
Daarom moet PCL 1.7.2 gecompileerd worden van hun github:
https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.7.2

Instructies om dit te compileren vind je hier:
http://pointclouds.org/downloads/source.html

Voor je de PCL kan compileren met CMake moeten eerst nog enkele dependencies geïnstalleerd worden:
- Boost
- Eigen
- FLANN
- VTK
- OpenNI (als je beelden van Kinectv1 of andere OpenNI devices wil kunnen opnemen)

Deze dependencies kunnen vanaf source gecompileerd worden maar makkelijker 
is om de installers voor visual studio 2010 te gebruiken op volgende link:
http://pointclouds.org/downloads/windows.html

OPGELET: bij het builden met CMake moet je de library builden als Release met "-DCMAKE_BUILD_TYPE=Release".
De debug libraries bevatten namelijk bugs in het NDT-algoritme en je krijgt runtime errors...

Zorg er ook voor dat alle dependencies dezelfde bit-versie hebben, allemaal 64-bit of allemaal 32-bit.

Na het uitvoeren van de cmake opdracht het resulterend visual studio 2010 project openen en builden.

<b>Linux:</b>

Zelfde als bij Windows maar na de cmake opdracht gewoon compileren met make en install. De dependencies moet je zelf installeren (makkelijkst met package manager)

<b>C++Rest SDK installeren:</b>

Windows:

C++ REST SDK 1.2.0 for Visual Studio 2010 downloaden en installeren van volgende link:
https://casablanca.codeplex.com/releases/view/111094

Ubuntu 12.04:

https://casablanca.codeplex.com/wikipage?title=Setup%20and%20Build%20on%20Linux%20%281.4%29&referringTitle=Setup%20and%20Build%20on%20Linux

Ubuntu 14.04:

https://casablanca.codeplex.com/wikipage?title=Setup%20and%20Build%20on%20Linux

<b>Projecten compileren en runnen</b>

CMakeLists van project runnen met CMake. Als gewerkt wordt met visual studio moet nog bij de instellingen verwezen worden naar de cpprest library, of je kan de verwijzing
eerst toevoegen in de CMakeLists door het juiste pad bij target_link_libraries toe te voegen (dit is noodzakelijk voor Linux).

Afhankelijk van de gebruikte compiler kunnen op Linux nog wat compilererrors optreden.

Aan het Client-project moet je 3 parameters meegeven: IP-adres van de client, IP-adres van de registrator en naam van de video die je wil versturen (videos worden opgeslaan als "voorbeeld0.pcd" "voorbeeld1.pcd" ... , je moet dus "voorbeeld" opgeven als parameter.

Aan het Cluster-project moet je 1 parameter meegeven: IP-adres van de registrator.

Aan het Registrator-project moet je 1 of meerdere IP-adressen meegeven van cluster servers.
