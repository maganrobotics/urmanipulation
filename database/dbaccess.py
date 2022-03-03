#!/usr

import MySQLdb as mdb
import numpy as np
from utils import dbcvt as dc
from panda3d.core import *

class GraspDB(object):

    def __init__(self):
        self.dbconnection = mdb.connect("localhost", "weiweilab", "weiweilab", "grasp")
        self.cursor = self.dbconnection.cursor()

    def execute(self, sql):
        """
        execute sql

        :param sql:
        :return: list if select, lastid if insert
        """

        try:
            self.cursor.execute(sql)
        except mdb.Error as e:
            print("MySQL Error [%d]: %s" % (e.args[0], e.args[1]))
            self.dbconnection.rollback()
            raise mdb.Error

        if sql[:3] == 'SEL':
            return list(self.cursor.fetchall())
        elif sql[:3] == 'INS':
            self.dbconnection.commit()
            return self.cursor.lastrowid
        else:
            self.dbconnection.commit()

    def unsafeexecute(self, sql):
        """
        execute sql

        :param sql:
        :return: list if select, lastid if insert
        """

        try:
            self.cursor.execute("SET FOREIGN_KEY_CHECKS = 0")
            self.cursor.execute("SET SQL_SAFE_UPDATES = 0")
            self.cursor.execute(sql)
            self.cursor.execute("SET SQL_SAFE_UPDATES = 1")
            self.cursor.execute("SET FOREIGN_KEY_CHECKS = 1")
        except mdb.Error as e:
            print( "MySQL Error [%d]: %s" % (e.args[0], e.args[1]))
            self.dbconnection.rollback()
            self.cursor.execute("SET SQL_SAFE_UPDATES = 1")
            self.cursor.execute("SET FOREIGN_KEY_CHECKS = 1")
            raise mdb.Error

        self.dbconnection.commit()

    def loadFreeAirGrip(self, objname, handname = "rtq85"):
        """
        load self.freegripid, etc. from mysqldatabase

        :param handname which hand to use, rtq85 by default
        :return: a list of [freegripid, freegripcontacts, freegripnormals, freegriprotmats, freegripjawwidth]


        """

        freegripid = []
        freegripcontacts = []
        freegripnormals = []
        freegriprotmats = []
        freegripjawwidth = []
        # access to db

        sql = "SELECT freeairgrip.idfreeairgrip, freeairgrip.contactpnt0, freeairgrip.contactpnt1, \
                freeairgrip.contactnormal0, freeairgrip.contactnormal1, freeairgrip.rotmat, \
                freeairgrip.jawwidth FROM freeairgrip, hand, object \
                WHERE freeairgrip.idobject = object.idobject AND object.name like '%s' \
                AND freeairgrip.idhand = hand.idhand AND hand.name like '%s'" % (objname, handname)
        data = self.execute(sql)
        if len(data) != 0:
            for i in range(len(data)):
                freegripid.append(int(data[i][0]))
                freegripcontacts.append([dc.strToV3(data[i][1]), dc.strToV3(data[i][2])])
                freegripnormals.append([dc.strToV3(data[i][3]), dc.strToV3(data[i][4])])
                freegriprotmats.append(dc.strToMat4(data[i][5]))
                freegripjawwidth.append(float(data[i][6]))

            return [freegripid, freegripcontacts, freegripnormals, freegriprotmats, freegripjawwidth]
        else:
            return None

    def loadIdHand(self, handname):
        sql = "SELECT idhand FROM hand WHERE name = '%s'" % handname
        result = self.execute(sql)
        print(result)
        if len(result) != 0:
            idhand = int(result[0][0])
        else:
            sql = "INSERT INTO hand(name) VALUES('%s')" % handname
            idhand = self.execute(sql)
        return idhand

    def loadIdObject(self, objname):
        sql = "SELECT idobject FROM object WHERE name = '%s'" % objname
        result = self.execute(sql)
        if len(result) != 0:
            idobj = int(result[0][0])
        else:
            sql = "INSERT INTO object(name) VALUES('%s')" % objname
            idobj = self.execute(sql)
        return idobj

if __name__ == '__main__':
    gdb = GraspDB()
    gdb.loadIdHand('rtq85')