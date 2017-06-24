'''
Created on Jun 19, 2017

@author: broihier
'''
import sqlite3

class DictDatabase(object):
    '''
    Dictionary based database using SQLite3.
    '''
    cursor = None

    def __init__(self, name):
        '''
        Constructor
        '''
        self.database = sqlite3.connect(name)
        self.database.row_factory = sqlite3.Row
        self.tables_and_columns = {}

    def drop(self, table):
        '''
        Delete a table
        '''
        try:
            self.database.execute("drop table {}".format(table))
        except sqlite3.OperationalError as error:
            if "{}".format(error).find("no such table") == -1:
                print("Table delete error: {}".format(error))

    def create(self, table_and_columns):
        '''
        Create a database table and schema.
        '''
        assert isinstance(dict(), type(table_and_columns))
        try:
            cursor = self.database.execute("create table {} ( {} )".format(
                table_and_columns["name"],
                table_and_columns["fields"]))
            if cursor is not None:
                self.database.commit()
            else:
                print("Table creation failure {}".format(table_and_columns["name"]))
        except sqlite3.OperationalError as error:
            if "{}".format(error).find("already exists") == -1:
                print("During creation, received error: {}".format(error))

    def update(self, table_columns_values):
        '''
        Update a database row.
        '''
        try:
            cursor = self.database.execute(
                "update {} set {} = {} where {} == {}"
                .format(table_columns_values["name"],
                        table_columns_values["fieldName"],
                        table_columns_values["value"],
                        table_columns_values["selector"],
                        table_columns_values["selectorValue"]))
            if cursor is not None:
                self.database.commit()
            else:
                print("Table update failure: {}".format(table_columns_values))
        except sqlite3.OperationalError as error:
            print("During update of a table, received error: {}".format(error))

    def insert(self, table_columns_values, buffer=False):
        '''
        Insert a database row.
        '''
        try:
            command = "insert into {} ({}) values ({})".format(
                table_columns_values["name"],
                table_columns_values["fieldNames"],
                table_columns_values["values"])

            cursor = self.database.execute(command)
            if cursor is not None:
                if not buffer:
                    self.database.commit()
            else:
                print("Table insert failure: {}".format(table_columns_values))
        except sqlite3.OperationalError as error:
            print("During insert of a table row, received error: {}".format(error))
            print(table_columns_values)
            print(command)

    def retrieve(self, selector):
        '''
        Set an internal cursor to retrieve rows of a table.
        '''
        try:
            if selector.get("field") != None:
                self.cursor = self.database.execute(
                    "select * from {} where {} = {}".format(selector["name"],
                                                            selector["field"],
                                                            selector["value"]))
            else:
                self.cursor = self.database.execute(
                    "select * from {} ".format(selector["name"]))

            if self.cursor is None:
                print("retrieve failed, selector was: {}".format(selector))
        except sqlite3.OperationalError as error:
            print("During retrieve, received error: {}".format(error))


    def get_row(self):
        '''
        Get a row from the current cursor.
        '''
        row = None
        try:
            row = dict(self.cursor.fetchone())
        except TypeError:
            pass
        return row

    def delete(self, selector):
        '''
        Delete a row of a table.
        '''
        try:
            self.cursor = self.database.execute(
                "delete from {} where {} = {}".format(selector["name"],
                                                      selector["field"],
                                                      selector["value"]))
            if self.cursor is None:
                print("delete failed, selector was: {}".format(selector))
        except sqlite3.OperationalError as error:
            print("During retrieve, received error: {}".format(error))


    def close(self):
        '''
        Close database.
        '''
        self.database.close()

def main():
    '''
    Self test code - should all execute without error.
    '''
    dbase = DictDatabase("test.db")
    dbase.create(dict(name="track_names", fields="name text"))
    dbase.create(dict(name="day1", fields="latitude float, longitude float"))
    dbase.insert(dict(name="day1", fieldNames="latitude,longitude",
                      values="1.0,2.0"))
    dbase.insert(dict(name="day1", fieldNames="latitude,longitude", values="99.9,88.8"))
    dbase.delete(dict(name="day1", field="latitude", value="99.9"))
    dbase.retrieve(dict(name="day1"))
    done = False
    while not done:
        row = dbase.get_row()
        if row is None:
            done = True
        else:
            print(row)
    dbase.create(dict(name="dropTable", fields="t1 text,t2 float"))
    dbase.drop("dropTable")
    dbase.drop("dropTable")
    dbase.close()
    print("done")

if __name__ == "__main__":
    main()
