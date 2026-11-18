# Sourced by the start scripts: picks the CycloneDDS configuration file to mount.
# Priority: CYCLONE_DDS_CONF already exported > ~/.config/cyclone_dds_settings.xml > repository fallback.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DDS_CONF_FALLBACK="$(realpath "${SCRIPT_DIR}/../../../app/navigation2/conf/cyclone_dds_sim_settings.xml")"

if [ -n "${CYCLONE_DDS_CONF:-}" ] && [ ! -f "${CYCLONE_DDS_CONF}" ]; then
    echo "CycloneDDS configuration file not found: ${CYCLONE_DDS_CONF}"
    unset CYCLONE_DDS_CONF
fi

if [ -z "${CYCLONE_DDS_CONF:-}" ]; then
    if [ -f "${HOME}/.config/cyclone_dds_sim_settings.xml" ]; then
        export CYCLONE_DDS_CONF="${HOME}/.config/cyclone_dds_sim_settings.xml"
    else
        export CYCLONE_DDS_CONF="${DDS_CONF_FALLBACK}"
    fi
fi

echo "Using the CycloneDDS configuration file in: ${CYCLONE_DDS_CONF}"
