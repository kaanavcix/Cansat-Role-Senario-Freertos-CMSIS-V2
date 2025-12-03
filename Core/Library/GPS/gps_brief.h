/*
 * gps_brief.h
 *
 *  Created on: Jul 23, 2025
 *      Author: kaana
 */

#ifndef LIBRARY_GPS_GPS_BRIEF_H_
#define LIBRARY_GPS_GPS_BRIEF_H_


/**
 * \defgroup        LWGPS_OPT Configuration
 * \brief           Default configuration setup
 * \{
 */

/**
 * \brief           Enables `1` or disables `0` `double precision` for floating point
 *                  values such as latitude, longitude, altitude.
 *
 *                  `double` is used as variable type when enabled, `float` when disabled.
 */
#ifndef LWGPS_CFG_DOUBLE
#define LWGPS_CFG_DOUBLE                    1
#endif

/**
 * \brief           Enables `1` or disables `0` status reporting callback
 *                  by \ref lwgps_process
 *
 * \note            This is an extension, so not enabled by default.
 */
#ifndef LWGPS_CFG_STATUS
#define LWGPS_CFG_STATUS                    0
#endif

/**
 * \brief           Enables `1` or disables `0` `GGA` statement parsing.
 *
 * \note            This statement must be enabled to parse:
 *                      - Latitude, Longitude, Altitude
 *                      - Number of satellites in use, fix (no fix, GPS, DGPS), UTC time
 */
#ifndef LWGPS_CFG_STATEMENT_GPGGA
#define LWGPS_CFG_STATEMENT_GPGGA           1
#endif

/**
 * \brief           Enables `1` or disables `0` `GSA` statement parsing.
 *
 * \note            This statement must be enabled to parse:
 *                      - Position/Vertical/Horizontal dilution of precision
 *                      - Fix mode (no fix, 2D, 3D fix)
 *                      - IDs of satellites in use
 */
#ifndef LWGPS_CFG_STATEMENT_GPGSA
#define LWGPS_CFG_STATEMENT_GPGSA           1
#endif

/**
 * \brief           Enables `1` or disables `0` `RMC` statement parsing.
 *
 * \note            This statement must be enabled to parse:
 *                      - Validity of GPS signal
 *                      - Ground speed in knots and coarse in degrees
 *                      - Magnetic variation
 *                      - UTC date
 */
#ifndef LWGPS_CFG_STATEMENT_GPRMC
#define LWGPS_CFG_STATEMENT_GPRMC           1
#endif

/**
 * \brief           Enables `1` or disables `0` `GSV` statement parsing.
 *
 * \note            This statement must be enabled to parse:
 *                      - Number of satellites in view
 *                      - Optional details of each satellite in view. See \ref LWGPS_CFG_STATEMENT_GPGSV_SAT_DET
 */
#ifndef LWGPS_CFG_STATEMENT_GPGSV
#define LWGPS_CFG_STATEMENT_GPGSV           1
#endif

/**
 * \brief           Enables `1` or disables `0` detailed parsing of each
 *                  satellite in view for `GSV` statement.
 *
 * \note            When this feature is disabled, only number of "satellites in view" is parsed
 */
#ifndef LWGPS_CFG_STATEMENT_GPGSV_SAT_DET
#define LWGPS_CFG_STATEMENT_GPGSV_SAT_DET   0
#endif

/**
 * \brief           Enables `1` or disables `0` parsing and generation
 *                  of PUBX (uBlox) messages
 *
 *                  PUBX are a nonstandard ublox-specific extensions,
 *                  so disabled by default.
 */
#ifndef LWGPS_CFG_STATEMENT_PUBX
#define LWGPS_CFG_STATEMENT_PUBX            0
#endif

/**
 * \brief           Enables `1` or disables `0` parsing and generation
 *                  of PUBX (uBlox) TIME messages.
 *
 * \note            TIME messages can be used to obtain:
 *                      - UTC time of week
 *                      - UTC week number
 *                      - Leap seconds (allows conversion to eg. TAI)
 *
 *                  This is a nonstandard ublox-specific extension,
 *                  so disabled by default.
 *
 *                  This configure option requires LWGPS_CFG_STATEMENT_PUBX
 */
#ifndef LWGPS_CFG_STATEMENT_PUBX_TIME
#define LWGPS_CFG_STATEMENT_PUBX_TIME       0
#endif

/**
 * \brief           Enables `1` or disables `0` CRC calculation and check
 *
 * \note            When not enabled, CRC check is ignored
 */
#ifndef LWGPS_CFG_CRC
#define LWGPS_CFG_CRC                       1
#endif

/* Guard against accidental parser breakage */
#if LWGPS_CFG_STATEMENT_PUBX_TIME && !LWGPS_CFG_STATEMENT_PUBX
#error LWGPS_CFG_STATEMENT_PUBX must be enabled when enabling LWGPS_CFG_STATEMENT_PUBX_TIME
#endif /* LWGPS_CFG_STATEMENT_PUBX_TIME && !LWGPS_CFG_STATEMENT_PUBX */

/**
 * \}
 */


#endif /* LIBRARY_GPS_GPS_BRIEF_H_ */
