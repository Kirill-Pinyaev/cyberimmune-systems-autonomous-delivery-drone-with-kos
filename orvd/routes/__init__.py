from .blueprint import bp
from .api_routes import *
from .mqtt_routes import *
from .socket_routes import *
from extensions import mqtt_client, socketio, task_scheduler_client

def init_app(app):
    app.register_blueprint(bp)
    mqtt_client.init_app(app)
    socketio.init_app(app)
    task_scheduler_client.init_app(app)