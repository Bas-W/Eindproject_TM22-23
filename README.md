# eindprojectTM22_23
In dit project maken we een line following robot.
Je vindt in deze repository de bestanden om met een 3D printer de nodige stukken te vervaardigen alsook de code en een pdf waarin uitgelegd wordt hoe de hardware moet worden gemonteerd.

Hoe de server moet worden opgezet wordt niet uitgelegd.

Het project maakt gebruik van MQTT om te communiceren tussen de server en de robot. De robot stuurt data naar de server via MQTT, deze data wordt dan in een influxDB database gezet door een python script. De data kan dan weergegeven worden in een Grafana webpagina. De robot kan ook bediend worden vanuit de webpagina via MQTT berichten.
