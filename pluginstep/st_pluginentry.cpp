#include "st_pluginentry.h"
#include "st_steppluginmanager.h"

ST_PluginEntry::ST_PluginEntry()
{
    _stepPluginManager = new ST_StepPluginManager();
}

ST_PluginEntry::~ST_PluginEntry()
{
    delete _stepPluginManager;
}

QString ST_PluginEntry::getVersion() const
{
    return "0.1";
}

CT_AbstractStepPlugin* ST_PluginEntry::getPlugin() const
{
    return _stepPluginManager;
}

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
    Q_EXPORT_PLUGIN2(plug_stepv6, ST_PluginEntry)
#endif
