
from constants import MissionVerificationStatus

def parse_mission(mission: str) -> list:
    """
    Разбирает строку миссии на список команд.

    Args:
        mission (str): Строка миссии.

    Returns:
        list: Список команд миссии.
    """
    cmds = mission.split('&')
    for idx, cmd in enumerate(cmds):
        cmds[idx] = [cmd[0], *cmd[1:].split('_')]
    return cmds


def read_mission(file_str: str) -> tuple[list | None, str]:
    """
    Читает миссию из строки файла и преобразует её в список команд.

    Args:
        file_str (str): Содержимое файла миссии.

    Returns:
        tuple: Кортеж, содержащий список команд миссии и статус верификации миссии.

    Raises:
        Exception: Если файл не поддерживается версией WP.
    """
    missionlist=[]
    split_str = '\r\n' if '\r' in file_str else '\n'
    for i, line in enumerate(file_str.split(split_str)):
        if line == '':
            break
        if i==0:
            if not line.startswith('QGC WPL 110'):
                raise Exception('File is not supported WP version')
        else:
            linearray=line.split('\t')
            ln_index=int(linearray[0])
            ln_currentwp=int(linearray[1])
            ln_frame=int(linearray[2])
            ln_command=int(linearray[3])
            ln_param1=float(linearray[4])
            ln_param2=float(linearray[5])
            ln_param3=float(linearray[6])
            ln_param4=float(linearray[7])
            ln_param5=float(linearray[8])
            ln_param6=float(linearray[9])
            ln_param7=float(linearray[10])
            #ln_autocontinue=int(linearray[11].strip())
            
            if ln_index == 0 and ln_currentwp == 1 and ln_frame == 0:
                cmd = home_handler(lat=ln_param5, lon=ln_param6, alt=ln_param7)
            elif ln_command == 22:
                cmd = takeoff_handler(alt=ln_param7)
            elif ln_command == 16:
                if ln_param1 == 0.0:
                    cmd = waypoint_handler(lat=ln_param5, lon=ln_param6, alt=ln_param7)
                else:
                    return None, MissionVerificationStatus.NON_ZERO_DELAY_WAYPOINT
            elif ln_command == 183:
                cmd = servo_handler(number=ln_param1, pwm=ln_param2)
            elif ln_command == 21:
                if len(missionlist) != 0 and missionlist[0][0] == 'H':
                    drone_home = missionlist[0]
                else:
                    drone_home = None
                cmd = land_handler(lat=ln_param5, lon=ln_param6, alt=ln_param7, home=drone_home)
            elif ln_command == 93:
                if ln_param2 == ln_param3 == ln_param4 == 0.0:
                    cmd = delay_handler(delay=ln_param1)
                else:
                    return None, MissionVerificationStatus.WRONG_DELAY
            elif ln_command == 201:
                cmd = roi_handler(lat=ln_param5, lon=ln_param6, alt=ln_param7)
            else:
                return None, MissionVerificationStatus.UNKNOWN_COMMAND
            
            missionlist.append(cmd)
    return missionlist, MissionVerificationStatus.OK


def home_handler(lat: float, lon: float, alt: float) -> list:
    """
    Обрабатывает команду установки домашней позиции.

    Args:
        lat (float): Широта.
        lon (float): Долгота.
        alt (float): Высота.

    Returns:
        list: Команда домашней позиции.
    """
    lat = round(lat, 7)
    lon = round(lon, 7)
    alt = round(alt, 2)
    return ['H', str(lat), str(lon), str(alt)]


def takeoff_handler(alt: float) -> list:
    """
    Обрабатывает команду взлёта.

    Args:
        alt (float): Высота взлёта.

    Returns:
        list: Команда взлёта.
    """
    alt = round(alt, 2)
    return ['T', str(alt)]


def waypoint_handler(lat: float, lon: float, alt: float) -> list:
    """
    Обрабатывает команду путевой точки.

    Args:
        lat (float): Широта.
        lon (float): Долгота.
        alt (float): Высота.

    Returns:
        list: Команда путевой точки.
    """
    lat = round(lat, 7)
    lon = round(lon, 7)
    alt = round(alt, 2)
    return ['W', str(lat), str(lon), str(alt)]


def servo_handler(number: float, pwm: float) -> list:
    """
    Обрабатывает команду управления сервоприводом.

    Args:
        number (float): Номер сервопривода.
        pwm (float): Значение ШИМ.

    Returns:
        list: Команда управления сервоприводом.
    """
    return ['S', str(number), str(pwm)]


def land_handler(lat: float, lon: float, alt: float, home: list = None) -> list:
    """
    Обрабатывает команду посадки.

    Args:
        lat (float): Широта.
        lon (float): Долгота.
        alt (float): Высота.
        home (list, optional): Домашняя позиция. По умолчанию None.

    Returns:
        list: Команда посадки.
    """
    if home is None:
        ret_lat = lat
        ret_lon = lon
        ret_alt = alt
    else:
        ret_lat = float(home[1]) if lat == 0. else lat
        ret_lon = float(home[2]) if lon == 0. else lon
        ret_alt = float(home[3]) if alt == 0. else alt
    
    ret_lat = round(ret_lat, 7)
    ret_lon = round(ret_lon, 7)
    ret_alt = round(ret_alt, 2)
    
    return ['L', str(ret_lat), str(ret_lon), str(ret_alt)]


def roi_handler(lat: float, lon: float, alt: float) -> list:
    """
    Обрабатывает команду DO_SET_ROI.

    Args:
        lat (float): Широта.
        lon (float): Долгота.
        alt (float): Высота.

    Returns:
        list: Команда DO_SET_ROI.
    """
    lat = round(lat, 7)
    lon = round(lon, 7)
    alt = round(alt, 2)
    return ['I', str(lat), str(lon), str(alt)]


def delay_handler(delay: int) -> list:
    """
    Обрабатывает команду задержки.

    Args:
        delay (int): Время задержки в секундах.

    Returns:
        list: Команда задержки.
    """
    return ['D', str(delay)]


def encode_mission(mission_list: list) -> list:
    """
    Кодирует список команд миссии в строковый формат.

    Args:
        mission_list (list): Список команд миссии.

    Returns:
        list: Закодированный список команд миссии.
    """
    for idx, cmd in enumerate(mission_list):
        mission_list[idx] = f'{cmd[0]}' + '_'.join(cmd[1:])
    
    return mission_list