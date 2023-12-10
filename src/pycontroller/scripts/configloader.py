import configparser

walk_balance_conf_path = '/home/name/agen3/src/pycontroller/scripts/config/walk_balance.ini'
ball_tracker_conf_path = '/home/name/agen3/src/pycontroller/scripts/config/ball_tracker.ini'
walking_conf_path = '/home/name/agen3/src/pycontroller/scripts/config/walking.ini'


walk_balance_parser = configparser.ConfigParser()   
walk_balance_parser.read(walk_balance_conf_path)

ball_tracker_parser = configparser.ConfigParser()   
ball_tracker_parser.read(ball_tracker_conf_path)

walking_parser = configparser.ConfigParser()   
walking_parser.read(walking_conf_path)

print(walk_balance_parser.sections())

def saveWalkBalanceConf():
    with open('example.ini', 'w') as configfile:
        walk_balance_parser.write(configfile)

def reload_walk_balance_conf():
    walk_balance_parser.read(walk_balance_conf_path)

def read_walk_balance_conf(group, item):
    walk_balance_parser = configparser.ConfigParser()   
    walk_balance_parser.read(walk_balance_conf_path)

    print("read conf data "+ item +" : ")
    data = float(walk_balance_parser[group][item])
    print(data)
    return data

def read_ball_track_conf(group, item):
    ball_tracker_parser = configparser.ConfigParser()   
    ball_tracker_parser.read(ball_tracker_conf_path)

    print("read conf data "+ item +" : ")
    data = float(ball_tracker_parser[group][item])
    print(data)
    return data

def read_walking_conf(group, item):
    walking_parser = configparser.ConfigParser()   
    walking_parser.read(walking_conf_path)

    print("read conf data "+ item +" : ")
    data = float(walking_parser[group][item])
    print(data)
    return data