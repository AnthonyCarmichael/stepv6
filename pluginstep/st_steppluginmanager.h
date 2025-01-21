#ifndef ST_STEPPLUGINMANAGER_H
#define ST_STEPPLUGINMANAGER_H

#include "ct_abstractstepplugin.h"

class ST_StepPluginManager : public CT_AbstractStepPlugin
{
public:
    ST_StepPluginManager();
    ~ST_StepPluginManager() override;

    QString getPluginURL() const override {return QString("No link available yet!");}

    virtual QString getPluginOfficialName() const override {return "Stepv6_ravaglia";}

    QString getPluginRISCitation() const override;
protected:

    bool loadGenericsStep() override;
    bool loadOpenFileStep() override;
    bool loadCanBeAddedFirstStep() override;
    bool loadActions() override;
    bool loadExporters() override;
    bool loadReaders() override;
};

#endif // ST_STEPPLUGINMANAGER_H
