FROM nodered/node-red:latest

# El directorio de trabajo por defecto es /usr/src/node-red.
# Instalamos los paquetes aquí para que formen parte de la imagen base
# y no se borren cuando se monte el volumen /data.
RUN npm install node-red-contrib-influxdb @flowfuse/node-red-dashboard