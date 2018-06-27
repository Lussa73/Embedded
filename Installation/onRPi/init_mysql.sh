mysql -uroot -e "CREATE DATABASE capteur_multi_pollutions"
mysql -uroot -e "CREATE USER 'Raspi'@'localhost' IDENTIFIED BY 'Raspi'"
mysql -uroot -e "GRANT ALL PRIVILEGES ON * . * TO 'Raspi'@'localhost' WITH GRANT OPTION"
mysql -uroot -e "CREATE USER 'Admin'@'%' IDENTIFIED BY 'Admin'"
mysql -uroot -e "GRANT ALL PRIVILEGES ON * . * TO 'Admin'@'%' WITH GRANT OPTION"
mysql -uroot -e "CREATE USER 'AppAndroid'@'%' IDENTIFIED BY 'AppAndroid'"
mysql -uroot -e "GRANT ALL PRIVILEGES ON capteur_multi_pollutions.* TO 'AppAndroid'@'%'"
mysql -uroot -e "CREATE USER 'Sensor'@'%' IDENTIFIED BY 'Sensor'"
mysql -uroot -e "GRANT ALL PRIVILEGES ON capteur_multi_pollutions.* TO 'Sensor'@'%'"
mysql -uroot capteur_multi_pollutions < /mysqldump.sql
