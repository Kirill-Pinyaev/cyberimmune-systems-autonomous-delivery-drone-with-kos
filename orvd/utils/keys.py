from hashlib import sha256
from Cryptodome import Random
from Cryptodome.PublicKey import RSA
from db.dao import get_key
from context import context
from constants import ORVD_KEY_SIZE, KeyGroup

def get_sha256_hex(message: str) -> str:
    """
    Вычисляет хеш SHA-256 для заданного сообщения и возвращает хэш в виде шестнадцатеричной строки.

    Args:
        message (str): Входное сообщение для хеширования.

    Returns:
        str: Хеш SHA-256 входного сообщения в виде шестнадцатеричной строки, без префикса '0x'.
    """
    return hex(int.from_bytes(sha256(message.encode()).digest(), byteorder='big', signed=False))[2:]


def sign(message: str, key_group: str) -> int:
    """
    Подписывает сообщение с использованием приватного ключа.

    Args:
        message (str): Сообщение для подписи.
        key_group (str): Группа ключей.

    Returns:
        int: Цифровая подпись.
    """
    key = get_key(key_group, private=True)
    n, d = key.n, key.d
    msg_bytes = message.encode()
    hash = int.from_bytes(sha256(msg_bytes).digest(), byteorder='big', signed=False)
    signature = pow(hash, d, n)
    
    return signature


def verify(message: str, signature: int, key_group: str) -> bool:
    """
    Проверяет подпись сообщения.
    
    Args:
        message (str): Проверяемое сообщение.
        signature (int): Цифровая подпись.
        key_group (str): Группа ключей.

    Returns:
        bool: True, если подпись верна, иначе False.
    """
    try:
        key_set = get_key(key_group, private=False)
        if len(key_set) == 2:
            n, e = key_set
        else:
            return False
        msg_bytes = message.encode()
        hash = int.from_bytes(sha256(msg_bytes).digest(), byteorder='big', signed=False)
        hashFromSignature = pow(signature, e, n)
        return hash == hashFromSignature
    except Exception:
        return False


def mock_verifier(*args, **kwargs):
    """
    Мок-функция для проверки подписи. Всегда возвращает True.

    Returns:
        bool: True
    """
    return True

def generate_keys(keysize: int, key_group: str) -> list:
    """
    Генерирует пару ключей RSA.

    Args:
        keysize (int): Размер ключа.
        key_group (str): Группа ключей.

    Returns:
        list: Сгенерированные ключи.
    """
    random_generator = Random.new().read
    key = RSA.generate(keysize, random_generator)
    context.loaded_keys[key_group] = key

def generate_orvd_keys() -> list:
    return generate_keys(ORVD_KEY_SIZE, KeyGroup.ORVD)