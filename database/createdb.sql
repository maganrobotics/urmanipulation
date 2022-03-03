CREATE SCHEMA grasp;
USE grasp;
CREATE TABLE hand (
	idhand int(11) NOT NULL AUTO_INCREMENT,
    name varchar(20) NOT NULL,
    PRIMARY KEY(idhand)
);
CREATE TABLE object (
	idobject int(11) NOT NULL AUTO_INCREMENT,
    name varchar(20) NOT NULL,
    PRIMARY KEY(idobject)
);
CREATE TABLE freeairgrip (
	idfreeairgrip int(11) NOT NULL AUTO_INCREMENT,
    contactpnt0 varchar(120) NOT NULL,
    contactpnt1 varchar(120) NOT NULL,
    contactnormal0 varchar(120) NOT NULL,
    contactnormal1 varchar(120) NOT NULL,
    rotmat varchar(360) NOT NULL,
    jawwidth varchar(45) NOT NULL,
    idobject int(11) NOT NULL,
    idhand int(11) NOT NULL,
    PRIMARY KEY (idfreeairgrip),
    FOREIGN KEY(idhand) REFERENCES hand(idhand)
		ON DELETE CASCADE ON UPDATE NO ACTION,
	FOREIGN KEY(idobject) REFERENCES object(idobject)
		ON DELETE CASCADE ON UPDATE NO ACTION
);