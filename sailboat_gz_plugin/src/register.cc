#include <gz/plugin/Register.hh>

#include "sailboat_gz_plugin/ForcePlugin.hh"

GZ_ADD_PLUGIN(
    sailboat_gz_plugin::ForcePlugin,
    gz::sim::System,
    sailboat_gz_plugin::ForcePlugin::ISystemConfigure,
    sailboat_gz_plugin::ForcePlugin::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(
    sailboat_gz_plugin::ForcePlugin,
    "sailboat_gz_plugin::ForcePlugin"
)