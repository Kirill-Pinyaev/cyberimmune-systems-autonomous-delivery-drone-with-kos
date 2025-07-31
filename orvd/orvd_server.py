import os
import logging
from flask import Flask

from context import context
from extensions import swagger
from db import clean_db, generate_user, create_all, init_app as db_init_app
from handlers import init_app as handlers_init_app
from routes import init_app as routes_init_app
from constants import log_level_map
from utils import generate_orvd_keys

class FlaskConfig:
    SQLALCHEMY_ECHO = False
    SQLALCHEMY_TRACK_MODIFICATIONS = False
    SQLALCHEMY_DATABASE_URI = 'sqlite:///orvd.db'
    SWAGGER = {
        'title': 'ORVD API',
        'uiversion': 3
    }

def create_app():
    app = Flask(__name__)
    app.config.from_object(FlaskConfig)
    
    app_log_level_str = os.getenv("APP_LOG_LEVEL", "INFO").upper()
    context.log_level = log_level_map.get(app_log_level_str, logging.INFO)
    
    swagger.init_app(app)
    db_init_app(app)
    handlers_init_app(app)
    routes_init_app(app)
    
    generate_orvd_keys()
    
    return app

def clean_app_db(app):
    with app.app_context():
        create_all()
        clean_db()
        generate_user()


if __name__ == "__main__":
    app = create_app()
    app.run()
