import sys
from db.dao import check_user_token

def bad_request(message: str):
    """
    Возвращает сообщение об ошибке с кодом 400 (Bad Request).

    Args:
        message (str): Сообщение об ошибке.

    Returns:
        tuple: Кортеж с сообщением об ошибке и кодом состояния 400.
    """
    return message, 400


def signed_request(handler_func, verifier_func, signer_func, query_str: str, key_group: str, sig: str, **kwargs):
    """
    Обрабатывает подписанный запрос, проверяя подпись и выполняя указанную функцию-обработчик.

    Args:
        handler_func (callable): Функция-обработчик запроса.
        verifier_func (callable): Функция для проверки подписи.
        signer_func (callable): Функция для подписи ответа.
        query_str (str): Строка запроса.
        key_group (str): Группа ключей.
        sig (str): Подпись запроса.
        **kwargs: Дополнительные аргументы для функции-обработчика.

    Returns:
        tuple: Кортеж с ответом и кодом состояния.
    """
    if sig is not None and verifier_func(query_str, int(sig, 16), key_group):
        answer = handler_func(**kwargs)
        ret_code = 200
    else:
        print(f'failed to verify {query_str}', file=sys.stderr)
        answer = '$Signature verification fail'
        ret_code = 403
    answer = f'{answer}#{hex(signer_func(answer, "orvd"))[2:]}'
    return answer, ret_code


def authorized_request(handler_func, token: str, **kwargs):
    """
    Обрабатывает авторизованный запрос, проверяя токен и выполняя указанную функцию-обработчик.

    Args:
        handler_func (callable): Функция-обработчик запроса.
        token (str): Токен авторизации.
        **kwargs: Дополнительные аргументы для функции-обработчика.

    Returns:
        tuple: Кортеж с ответом и кодом состояния.
    """
    if check_user_token(token):
        answer = handler_func(**kwargs)
        ret_code = 200
    else:
        answer = '$Unauthorized'
        ret_code = 401
    return answer, ret_code


def regular_request(handler_func, **kwargs):
    """
    Обрабатывает обычный запрос, выполняя указанную функцию-обработчик.

    Args:
        handler_func (callable): Функция-обработчик запроса.
        **kwargs: Дополнительные аргументы для функции-обработчика.

    Returns:
        tuple: Кортеж с ответом и кодом состояния.
    """
    try:
        answer = handler_func(**kwargs)
        ret_code = 200
    except Exception:
        answer = 'Conflict.'
        ret_code = 409
    return answer, ret_code