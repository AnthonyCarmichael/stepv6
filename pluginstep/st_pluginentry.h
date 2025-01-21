#ifndef ST_PLUGINENTRY_H
#define ST_PLUGINENTRY_H

#include "pluginentryinterface.h"

class ST_StepPluginManager;

class ST_PluginEntry : public PluginEntryInterface
{
    Q_OBJECT

#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
    Q_PLUGIN_METADATA(IID PluginEntryInterface_iid)
#endif

    Q_INTERFACES(PluginEntryInterface)

public:
    ST_PluginEntry();
    ~ST_PluginEntry() override;

    QString getVersion() const override;
    CT_AbstractStepPlugin* getPlugin() const override;

private:
    ST_StepPluginManager *_stepPluginManager;
};

#endif // ST_PLUGINENTRY_H
