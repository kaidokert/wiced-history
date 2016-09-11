/*
 * Copyright 2014, Broadcom Corporation
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */
#pragma once

/** @file
 *  Defines SmartBridge Generic Attribute Profile (GATT) Functions
 */

#include "wiced_utilities.h"
#include "wiced_bt_smartbridge.h"
#include "wiced_bt_smart_interface.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/*****************************************************************************/
/** @addtogroup sbgatt SmartBridge GATT Procedures
 *  @ingroup smartbridge
 *
 *  SmartBridge Raw GATT Functions
 *
 *
 *  @{
 */
/*****************************************************************************/


/** Discover All Primary Services
 *
 * @param[in]  socket       : socket that is connected to the server to discover
 *                            Primary Services from
 * @param[out] service_list : pointer that will receive the list of Primary Services
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_bt_smartbridge_gatt_discover_all_primary_services( const wiced_bt_smartbridge_socket_t* socket, wiced_bt_smart_attribute_list_t* service_list );


/** Discover Primary Services by the given UUID
 *
 * @param[in]  socket       : socket that is connected to the server to discover
 *                            Primary Services from
 * @param[in]  uuid         : unique identifier of the Primary Services to discover
 * @param[out] service_list : pointer that will receive the list of Primary Services
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_bt_smartbridge_gatt_discover_primary_services_by_uuid( const wiced_bt_smartbridge_socket_t* socket, const wiced_bt_uuid_t* uuid, wiced_bt_smart_attribute_list_t* service_list );


/** Find Included Services
 *
 * @param[in]  socket       : socket that is connected to the server to discover
 *                            Included Services from
 * @param[in]  start_handle : starting Attribute handle of the Primary Service to
 *                            find the Included Services from
 * @param[in]  end_handle   : ending Attribute handle of the Primary Service to
 *                            find the Included Services from
 * @param[out] include_list : pointer that will receive the list of Included Services
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_bt_smartbridge_gatt_find_included_services( const wiced_bt_smartbridge_socket_t* socket, uint16_t start_handle, uint16_t end_handle, wiced_bt_smart_attribute_list_t* include_list );


/** Discover All Characterisitics in a Service
 *
 * @param[in]  socket              : socket that is connected to the server to discover
 *                                   Characteristics from
 * @param[in]  start_handle        : starting Attribute handle of the Primary Service to
 *                                   discover Characteristics from
 * @param[in]  end_handle          : ending Attribute handle of the Primary Service to
 *                                   discover Characteristics from
 * @param[out] characteristic_list : pointer that will receive the list of Characteristics
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_bt_smartbridge_gatt_discover_all_characteristics_in_a_service( const wiced_bt_smartbridge_socket_t* socket, uint16_t start_handle, uint16_t end_handle, wiced_bt_smart_attribute_list_t* characteristic_list );


/** Discover Characterisitics by the given UUID
 *
 * @param[in]  socket              : socket that is connected to the server to discover
 *                                   Characteristics from
 * @param[in]  uuid                : unique identifier of the Characteristics to discover
 * @param[in]  start_handle        : starting Attribute handle of the Primary Service to
 *                                   discover Characteristics from
 * @param[in]  end_handle          : ending Attribute handle of the Primary Service to
 *                                   discover Characteristics from
 * @param[out] characteristic_list : pointer that will receive the list of Characteristics
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_bt_smartbridge_gatt_discover_characteristic_by_uuid( const wiced_bt_smartbridge_socket_t* socket, const wiced_bt_uuid_t* uuid, uint16_t start_handle, uint16_t end_handle, wiced_bt_smart_attribute_list_t* characteristic_list );


/** Discover Attribute Handle and Type of all Characteristic Descriptors
 *
 * @note
 * Additional information of the Descriptors can be read using @ref wiced_bt_smartbridge_gatt_read_characteristic_descriptor()
 * and @ref wiced_bt_smartbridge_gatt_read_long_characteristic_descriptor()
 *
 * @param[in]  socket          : socket that is connected to the server to discover
 *                               Characteristic Descriptors from
 * @param[in]  start_handle    : starting Attribute handle of the Characteristic to
 *                               discover Characteristic Descriptors from
 * @param[in]  end_handle      : ending Attribute handle of the Characteristic to
 *                               discover Characteristic Descriptors from
 * @param[out] descriptor_list : pointer that will receive the list of Descriptors
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_bt_smartbridge_gatt_discover_handle_and_type_of_all_characteristic_descriptors( const wiced_bt_smartbridge_socket_t* socket, uint16_t start_handle, uint16_t end_handle, wiced_bt_smart_attribute_list_t* descriptor_list );


/** Read Characteristic Descriptor
 *
 * @param[in]  socket     : socket that is connected to the server to read
 *                          Characteristic Descriptor  from
 * @param[in]  handle     : Attribute handle of the Characteristic Descriptor to read
 * @param[in]  uuid       : unique identifier of the Characteristic Descriptor to read
 * @param[out] descriptor : pointer that will receive the Descriptors
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_bt_smartbridge_gatt_read_characteristic_descriptor( const wiced_bt_smartbridge_socket_t* socket, uint16_t handle, const wiced_bt_uuid_t* uuid, wiced_bt_smart_attribute_t** descriptor );


/** Read Long Characteristic Descriptor
 *
 * @param[in]  socket     : socket that is connected to the server to read
 *                          Characteristic Descriptor from
 * @param[in]  handle     : Attribute handle of the Characteristic Descriptor to read
 * @param[in]  uuid       : unique identifier of the Characteristic Descriptor to read
 * @param[out] descriptor : pointer that will receive the Descriptors
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_bt_smartbridge_gatt_read_long_characteristic_descriptor( const wiced_bt_smartbridge_socket_t* socket, uint16_t handle, const wiced_bt_uuid_t* uuid, wiced_bt_smart_attribute_t** descriptor );


/** Write Characteristic Descriptor
 *
 * @param[in]  socket    : socket that is connected to the server to write
 *                         Characteristic Descriptor to
 * @param[in] descriptor : Characteristic Descriptor to write
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_bt_smartbridge_gatt_write_characteristic_descriptor( const wiced_bt_smartbridge_socket_t* socket, const wiced_bt_smart_attribute_t* descriptor );


/** Write Long Characteristic Descriptor
 *
 * @param[in]  socket    : socket that is connected to the server to write
 *                         Characteristic Descriptor to
 * @param[in] descriptor : Characteristic Descriptor to write
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_bt_smartbridge_gatt_write_long_characteristic_descriptor( const wiced_bt_smartbridge_socket_t* socket, const wiced_bt_smart_attribute_t* descriptor );


/** Read Characteristic Value
 *
 * @param[in]  socket               : socket that is connected to the server to read
 *                                    Characteristic Value from
 * @param[in]  handle               : Attribute handle of the Characteristic Value to read
 * @param[in]  uuid                 : unique identifier of the Characteristic Value to read
 * @param[out] characteristic_value : pointer that will receive the Characteristic Value
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_bt_smartbridge_gatt_read_characteristic_value( const wiced_bt_smartbridge_socket_t* socket, uint16_t handle, const wiced_bt_uuid_t* uuid, wiced_bt_smart_attribute_t** characteristic_value );


/** Read Characteristic Value
 *
 * @param[in]  socket                    : socket that is connected to the server to read
 *                                         Characteristic Value from
 * @param[in]  uuid                      : unique identifier of the Characteristic Values to read
 * @param[out] characteristic_value_list : pointer that will receive the Characteristic Value list
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_bt_smartbridge_gatt_read_characteristic_values_using_uuid( const wiced_bt_smartbridge_socket_t* socket,  const wiced_bt_uuid_t* uuid, wiced_bt_smart_attribute_list_t* characteristic_value_list );


/** Read Long Characteristic Value
 *
 * @param[in]  socket               : socket that is connected to the server to read
 *                                    Characteristic Value from
 * @param[in]  handle               : Attribute handle of the Characteristic Value to read
 * @param[in]  uuid                 : unique identifier of the Characteristic Value to read
 * @param[out] characteristic_value : pointer that will receive the Characteristic Value
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_bt_smartbridge_gatt_read_long_characteristic_value( const wiced_bt_smartbridge_socket_t* socket, uint16_t handle, const wiced_bt_uuid_t* uuid, wiced_bt_smart_attribute_t** characteristic_value );


/** Write Characteristic Value
 *
 * @param[in]  socket               : socket that is connected to the server to write
 *                                    Characteristic Value to
 * @param[in]  characteristic_value : Characteristic Value to write
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_bt_smartbridge_gatt_write_characteristic_value( const wiced_bt_smartbridge_socket_t* socket, const wiced_bt_smart_attribute_t* characteristic_value );


/** Write Long Characteristic Value
 *
 * @param[in]  socket              : socket that is connected to the server to write
 *                                   Characteristic Value to
 * @param[in] characteristic_value : Characteristic Value to write
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_bt_smartbridge_gatt_write_long_characteristic_value( const wiced_bt_smartbridge_socket_t* socket, const wiced_bt_smart_attribute_t* characteristic_value );


/** @} */
