/* Copyright 2015 The MathWorks, Inc. */

#include "slros_generic_param.h"

/**
 * Initialize the parameter getter class.
 * @param pName The name of the ROS parameter
 */
void SimulinkParameterGetterBase::initialize(const std::string& pName)
{
    nodePtr = SLROSNodePtr;
    paramName = pName;
    hasValidValue = false;    
}

/**
* Initialize the constants for the error codes.
* @param codeSuccess Error code that should be emitted if parameter is retrieved successfully
* @param codeNoParam Error code if parameter with given name does not exist on server
* @param codeTypeMismatch Error code if parameter exists on server, but has a different data type
* @param codeArrayTruncate Error code if received array was truncated
*/
void SimulinkParameterGetterBase::initialize_error_codes(
        uint8_t codeSuccess, uint8_t codeNoParam, uint8_t codeTypeMismatch,
        uint8_t codeArrayTruncate)
{
    // Initialize the error codes
    errorCodeSuccess = codeSuccess;
    errorCodeNoParam = codeNoParam;
    errorCodeTypeMismatch = codeTypeMismatch;
    errorCodeArrayTruncate = codeArrayTruncate;
}