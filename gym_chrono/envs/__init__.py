"""gym_chrono.envs — environment implementations.

Submodules import :class:`~gym_chrono.envs.ChronoBase.ChronoBaseEnv` directly
from ``gym_chrono.envs.ChronoBase``; this package ``__init__`` stays empty so
minimal envs (e.g. Chrono 10 Viper walk) can load without pulling optional
Chrono modules through unrelated imports.
"""
