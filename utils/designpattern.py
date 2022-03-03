

def singleton(cls):
    '''
    generate only one instance of the class
    things are saved in a dictionary named instances

    ## input
    cls:
        the name of the class

    ## output
        the single instance of the cls


    '''
    instances = {}
    def getinstance():
        if cls not in instances:
            instances[cls] = cls()
        return instances[cls]
    return getinstance()