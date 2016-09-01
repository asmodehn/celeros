# Ref : http://loose-bits.com/2010/12/celery-logging-with-python-logging.html
# TODO : probably a better solution if we can setup handlers before starting celery worker
# While telling celery not to hijack existing root logger
import logging

import celery.app.log

from celery.app.log import ColorFormatter
from . import config

#
# def _configure_logger(self, logger, logfile, loglevel,
#                       format, colorize, **kwargs):
#     if logger is not None:
#         self.setup_handlers(logger, logfile, format,
#                             colorize, **kwargs)
#         if loglevel:
#             logger.setLevel(loglevel)
#
#
# def setup_handlers(self, logger, logfile, format, colorize,
#                    formatter=ColorFormatter, **kwargs):
#     if self._is_configured(logger):
#         return logger
#     handler = self._detect_handler(logfile)
#     handler.setFormatter(formatter(format, use_color=colorize))
#     logger.addHandler(handler)
#     return logger


old_configure_logger = celery.app.log.Logging._configure_logger  # Store the real method.
old_setup_handlers = celery.app.log.Logging.setup_handlers  # Store the real method.


def celeros_configure_logger(self, logger, logfile, loglevel,
                      format, colorize, **kwargs):
    """Replacement for :method:`celery.app.log._configure_logger`."""
    if logger is not None:
        self.setup_handlers(logger, logfile, format,
                            colorize, **kwargs)
        if loglevel:
            logger.setLevel(loglevel)


def celeros_setup_handlers(self, logger, logfile, format, colorize,
                   formatter=ColorFormatter, **kwargs):
    """Replacement for :method:`celery.app.log.setup_handlers`."""

    patched_logger = old_setup_handlers(self,
        logger, logfile, format, colorize, formatter, **kwargs)

    # Check if not patched.
    if not getattr(patched_logger, "_LOG_PATCH_DONE", False):
        # Lock and patch.
        logging._acquireLock()
        try:
            handlers = getattr(config, "PATCH_CELERYD_LOG_EXTRA_HANDLERS", [])
            for handler_lvl, handler_fn in handlers:
                handler = handler_fn()
                handler.setLevel(handler_lvl)
                handler.setFormatter(formatter(format, use_color=colorize))
                patched_logger.addHandler(handler)

            # Mark logger object patched.
            setattr(patched_logger, "_LOG_PATCH_DONE", True)
        finally:
            logging._releaseLock()


    return patched_logger


# Apply patches.
CELERY_MOD_PATCHED = False
if not CELERY_MOD_PATCHED:
    logging._acquireLock()  # Lock logging during patch.
    try:
        celery.app.log.Logging._configure_logger = celeros_configure_logger  # Patch old w/ new.
        celery.app.log.Logging.setup_handlers = celeros_setup_handlers  # Patch old w/ new.
        CELERY_MOD_PATCHED = True
    finally:
        logging._releaseLock()
