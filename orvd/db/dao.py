import os
import secrets
from extensions import db
from hashlib import sha256
from constants import KeyGroup
from context import context
from .models import User, UavTelemetry, MissionStep, Mission, MissionSenderPublicKeys, UavPublicKeys, Uav, Event


def add_and_commit(entity: db.Model):
    """
    Добавляет сущность в сессию и фиксирует изменения.

    Args:
        entity (db.Model): Сущность для добавления и фиксации.

    Return:
        None
    """
    db.session.add(entity)
    db.session.commit()
    
    
def add_changes(entity: db.Model):
    """
    Добавляет сущность в сессию без фиксации изменений.

    Args:
        entity (db.Model): Сущность для добавления.

    Return:
        None
    """
    db.session.add(entity)
    
    
def commit_changes():
    """
    Фиксирует изменения в базе данных.

    Args:
        None

    Return:
        None
    """
    db.session.commit()
    
def flush():
    db.session.flush()
    
    
def delete_entity(entity: db.Model):
    """
    Удаляет сущность из сессии.

    Args:
        entity (db.Model): Сущность для удаления.

    Return:
        None
    """
    db.session.delete(entity)


def get_entity_by_key(entity: db.Model, key_value):
    """
    Получает сущность по ключевому значению.

    Args:
        entity (db.Model): Модель сущности.
        key_value: Значение ключа для поиска.

    Return:
        db.Model: Найденная сущность или None.
    """
    return entity.query.get(key_value)


def generate_user():
    """
    Создает пользователя-администратора и добавляет его в базу данных.

    Return:
        None
    """
    user_entity = User(username=str(os.getenv("ADMIN_LOGIN")),
                       password_hash=hex(int.from_bytes(sha256(str(os.getenv("ADMIN_PASSW")).encode()).digest(),
                                                        byteorder='big', signed=False))[2:],
                       access_token=secrets.token_hex(16))
    db.session.add(user_entity)
    db.session.commit()
    
def check_user_token(token: str):
    """
    Проверяет валидность токена пользователя.

    Args:
        token (str): Токен для проверки.

    Returns:
        bool: True, если токен валиден, иначе False.
    """
    users = get_entities_by_field(User, User.access_token, token)
    if users and users.count() != 0:
        return True
    else:
        return False

def get_entities_by_field(entity: db.Model, field, field_value, order_by_field=None) -> list:
    """
    Получает список сущностей по значению поля.

    Args:
        entity (db.Model): Модель сущности.
        field: Поле для фильтрации.
        field_value: Значение поля для фильтрации.
        order_by_field: Поле для сортировки (опционально).

    Return:
        list: Список найденных сущностей.
    """
    entities = entity.query.filter(field==field_value)
    if order_by_field is None:
        return entities
    else:
        return entities.order_by(order_by_field)
    

def get_entities_by_field_with_order(entity: db.Model, field, field_value, order_by_field) -> list:
    """
    Получает отсортированный список сущностей по значению поля.

    Args:
        entity (db.Model): Модель сущности.
        field: Поле для фильтрации.
        field_value: Значение поля для фильтрации.
        order_by_field: Поле для сортировки.

    Return:
        list: Список найденных сущностей.
    """
    return entity.query.filter(field==field_value).order_by(order_by_field)


def clean_db():
    """
    Очищает базу данных, удаляя все записи указанных моделей.

    Args:
        models_to_clean: Список моделей для очистки.

    Return:
        None
    """
    try:
        for model in [User, UavTelemetry, MissionStep, Mission, MissionSenderPublicKeys, UavPublicKeys, Uav, Event]:
            db.session.query(model).delete()
        db.session.commit()
    except Exception:
        db.session.rollback()

def create_all():
    try:
        db.create_all()
    except Exception:
        db.session.rollback()
    
def get_key(key_group: str, private: bool):
    """
    Получает ключ из указанной группы.

    Args:
        key_group (str): Группа ключей.
        private (bool): Флаг для получения приватного ключа.

    Returns:
        Ключ или кортеж (n, e) для публичного ключа, или -1 в случае ошибки.
    """
    if private is True:
        if key_group in context.loaded_keys:
            return context.loaded_keys[key_group]
        else:
            return None
    
    else:
        if KeyGroup.KOS in key_group:
            id = key_group.split(KeyGroup.KOS)[1]
            key = get_entity_by_key(UavPublicKeys, id)
            if key is None:
                return -1
            n, e = int(key.n), int(key.e)
            
        elif KeyGroup.MS in key_group:
            id = key_group.split(KeyGroup.MS)[1]
            key = get_entity_by_key(MissionSenderPublicKeys, id)
            if key is None:
                return -1
            n, e = int(key.n), int(key.e)
        
        elif key_group == KeyGroup.ORVD:
            key = context.loaded_keys[key_group].publickey()
            n, e = key.n, key.e
            
        else:
            print('Wrong group')
            return -1
        
        return n, e


def save_public_key(n: str, e: str, key_group: str) -> None:
    """
    Сохраняет публичный ключ в базу данных.

    Args:
        n (str): Модуль ключа.
        e (str): Открытая экспонента.
        key_group (str): Группа ключей.
    """
    if KeyGroup.KOS in key_group:
        id = key_group.split(KeyGroup.KOS)[1]
        entity = UavPublicKeys(uav_id=id, n=n, e=e)
    elif KeyGroup.MS in key_group:
        id = key_group.split(KeyGroup.MS)[1]
        entity = MissionSenderPublicKeys(uav_id=id, n=n, e=e)
    else:
        print('Wrong group in utils.save_public_key')
    add_and_commit(entity)


def save_event(uav_id: str, log_message: str) -> None:
    """
    Сохраняет событие в базу данных.

    Args:
        uav_id (str): Идентификатор БПЛА.
        log_message (str): Сообщение лога.
    """
    event = Event(uav_id=uav_id, log_message=log_message)
    add_and_commit(event)


def get_events_by_uav_id(uav_id: str) -> list:
    """
    Получает все события для указанного БПЛА.

    Args:
        uav_id (str): Идентификатор БПЛА.

    Returns:
        list: Список событий.
    """
    return get_entities_by_field(Event, Event.uav_id, uav_id, Event.timestamp).all()