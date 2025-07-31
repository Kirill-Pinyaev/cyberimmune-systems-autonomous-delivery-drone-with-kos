from extensions import db, migrate
from .dao import *
from .models import *

def init_app(app):
    db.init_app(app)
    migrate.init_app(app, db)