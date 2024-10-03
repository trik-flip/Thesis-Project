# dependency injection
import dependency_injector.containers as containers
from dependency_injector import providers


class Container(containers.DeclarativeContainer):
    rc = providers.Singleton(SimController)
