/* Copyright 2015 The MathWorks, Inc. */

#ifndef _SLROS_MSGCONVERT_UTILS_H_
#define _SLROS_MSGCONVERT_UTILS_H_

#include <algorithm>

extern const std::string SLROSNodeName; ///< Name of the Simulink ROS node


namespace slros
{
    
    /**
     * Class for handling warnings generated during roscpp <-> Simulink bus
     * conversions. This encapsulates how & if warnings are emitted.
     */
    class ROSPropertyWarnStatus
    {
    public:
        /**
         * Emit warning that a variable-length array is truncated during 
         * conversion from a roscpp message to a Simulink bus. By default, 
         * this method is a no-op (does not emit a warning).
         * 
         * @param numReceivedItems Number of items in the roscpp array property
         * @param maxArraySize Maximum number of items in the Simulink bus property
         */
        virtual void emitTruncationWarning(int numReceivedItems, int maxArraySize) const {};
    };
    
    
    /**
     * NoopWarning indicates that warnings are not meaningful. This should be used
     * to pass a ROSPropertyWarnStatus to a function that is not expected to issue any
     * warnings (e.g., function dealing with fixed-size arrays would not show
     * any truncation warnings).
     *
     * The implementation is empty since it does not change the default behavior 
     * of the base class.
     */      
    class NoopWarning : public ROSPropertyWarnStatus   {};
    
    
    /**
     * DisabledWarning indicates that warnings should be suppressed (i.e., should 
     * not be emitted). The behavior is the same as NoopWarning, but the intent is 
     * different (DisabledWarning indicates warnings are explicitly suppressed, 
     * whereas NoopWarning indicates that warnings are not meaningful).
     *
     * The implementation is empty since it does not change the default behavior 
     * of the base class.
     *     
     * @see EnabledWarning
     */    
    class DisabledWarning : public ROSPropertyWarnStatus {};
    
    
    /**
     * EnabledWarning indicates that warnings should be emitted in a ROS-approriate 
     * manner. This class expects SLROSNodeName to be defined (the name of the 
     * Simulink ROS node)
     *
     * @see DisabledWarning
     */           
    class EnabledWarning : public ROSPropertyWarnStatus
    {        
    public:        
        /**
         * Constructs a EnabledWarning object
         *
         * @param msgType Name of the ROS message type being manipulated
         * @param propertyName Name of the ROS property being manipulated
         */
        EnabledWarning(const std::string &msgType, const std::string &propertyName) :
            _messageType(msgType),
            _rosPropertyName(propertyName)
            {};
            
       inline void emitTruncationWarning(int numReceivedItems, int maxArraySize) const
       {
           ROS_WARN_NAMED(SLROSNodeName,
                   "Truncating array '%s' in received message '%s' from %d to %d items",
                   _rosPropertyName.c_str(), _messageType.c_str(), numReceivedItems, maxArraySize);
       }
    
    private:
        const std::string _messageType;
        const std::string _rosPropertyName;        
    };


    //-------------------------------------------------------
    // Utility functions
    
    /**
     * Return the type of a ROS message (e.g., "geometry_msgs/Point")
     *
     * @param msg ROS message
     * @retval char* Type of the ROS message as a zero-delimited string
     */    
    template <typename MsgType>
    inline const char *getROSMessageType(MsgType &msg)
    {
        return ros::message_traits::DataType<MsgType>::value();
    }
    
    /**
     * Calculate the number of items in a fixed-size simple array.
     * "Constant size" refers to an array that is declared like "MyType foo[N]"
     * (as opposed to a STL container like vector).
     * "Simple array" means that the size of an array items is completely known
     * at compile time (specifically, a vanilla C struct).
     *
     * @param array Reference to the array. Do not pass in a pointer!
     * @retval int The number of items in the array
     */
    template <typename ArrayType>
    inline int getNumItemsInFixedSimpleArray(ArrayType &array)
    {
        return (sizeof array) /  (sizeof array[0]);
    }        
    
    /**
     * When converting a roscpp message to a Simulink bus array, the 
     * ReceivedLength and CurrentLength values for the Simulink BusInfo
     * struct need to be set appropriately. This utility function sets  
     * these values and emits a truncation warning if needed. 
     *
     * Assumptions:
     * 1) The caller is responsible for the actual copying of the content 
     *    to the bus array (note that busProp is a const input)
     *
     * @param busProp[in]      Variable-length array property in Simulink bus
     * @param busInfoProp[out] Info property in Simulink bus corresponding to variable-length array
     * @param msgProp[in]      Variable-length array property in in roscpp message
     * @param warnStatus[in]   Handler for warnings during conversion
     * @retval int             The number of items to be copied (aka. CurrentLength)
     */    
    template <typename BusType, typename BusInfoType, typename MsgType>
    inline int setReceivedAndCurrentLengths(const BusType &busProp, BusInfoType &busInfoProp,
            const MsgType &msgProp, const slros::ROSPropertyWarnStatus &warnStatus)
    {
        const int numItemsReceived = msgProp.size();
        const int maxBusArraySize = slros::getNumItemsInFixedSimpleArray(busProp);
        const int numItemsToCopy = std::min(numItemsReceived, maxBusArraySize);
        
        busInfoProp.ReceivedLength = numItemsReceived;
        busInfoProp.CurrentLength = numItemsToCopy;
        
        if (numItemsReceived > maxBusArraySize) 
        {
            warnStatus.emitTruncationWarning(numItemsReceived, maxBusArraySize);
        }
        return numItemsToCopy;
    }
    
    
    /**
     * Utility function to convert strings in a string array from a roscpp 
     * message to Simulink bus property. This function modifies the SL_Info 
     * metadata for the individual strings.
     *
     * Assumptions: 
     * 1) The caller is responsible for modifying the SL_Info metadata for 
     *    the string array itself (e.g., the total number of strings in the 
     *    string array).
     * 2) numStringsToCopy has already been bounds-checked
     *
     * @param busProp[out]         Fixed-length string array property in Simulink bus
     * @param msgProp[in]          Fixed-length string array property in roscpp message
     * @param warnStatus[in]       Handler for warnings during conversion
     * @param numStringsToCopy[in] Number of strings to copy
     */
    template <typename BusType, typename MsgType>
            inline void convertToBusStringsInStringArray(BusType &busProp, const MsgType &msgProp,
            const slros::ROSPropertyWarnStatus &warnStatus, int numStringsToCopy)
    {        
        for (int i=0; i < numStringsToCopy; i++)
        {
            const int numCharsToCopy = slros::setReceivedAndCurrentLengths(
                    busProp[i].Data, busProp[i].Data_SL_Info, msgProp[i], warnStatus);
            std::copy(msgProp[i].begin(), msgProp[i].begin() + numCharsToCopy, busProp[i].Data);
        }
    }
   
    
    /**
     * Utility function to convert strings in a string array from a Simulink bus 
     * to a roscpp message property.
     *
     * Assumptions: 
     * 1) The caller is responsible for resizing the string array property
     *    to numStringsToCopy
     * 2) numStringsToCopy has already been bounds-checked
     *     
     * @param msgProp[out]         Fixed-length string array property in roscpp message
     * @param busProp[in]          Fixed-length string array property in Simulink bus
     * @param numStringsToCopy[in] Number of strings to copy
     */
    template <typename BusType, typename MsgType>
            inline void convertFromBusStringsInStringArray(MsgType &msgProp,
            const BusType &busProp, int numStringsToCopy)
    {
        for (int i=0; i < numStringsToCopy; i++)
        {
            const int numCharsToCopy = busProp[i].Data_SL_Info.CurrentLength;
            msgProp[i].resize(numCharsToCopy);
            std::copy(busProp[i].Data, busProp[i].Data + numCharsToCopy, msgProp[i].begin());
        }
    }
} // namespace slros


//====================================================================================
// Calls to the following conversion routines (e.g., convertToBusFixedNestedArray) 
// are generated in getBusToCppConversionFcns.m

        
//-------------------------------------------------------
// Conversion routines for arrays of nested messages

/**
 * Convert a single property (fixed-length nested array of messages) 
 * from a roscpp message to Simulink bus.
 *
 * @param busProp[out]   Fixed-length array property in Simulink bus
 * @param msgProp[in]    Fixed-length array property in roscpp message
 * @param warnStatus[in] Handler for warnings during conversion
 */
template <typename BusType, typename MsgType>
inline void convertToBusFixedNestedArray(BusType &busProp, MsgType &msgProp, 
        const slros::ROSPropertyWarnStatus &warnStatus)
{    
    const int numItemsToCopy = slros::getNumItemsInFixedSimpleArray(busProp);
    for (int i=0; i < numItemsToCopy; i++)
    {
        convertToBus(&busProp[i], &msgProp[i]);
    }    
}


/**
 * Convert a single property (variable-length nested array of messages) 
 * from a roscpp message to Simulink bus. When the roscpp array is shorter 
 * than the maximum length of the Simulink array, the remaining elements 
 * in the Simulink array left untouched (i.e., they are not explicitly zeroed
 * out).
 *
 * @param busProp[out]     Variable-length array property in Simulink bus
 * @param busInfoProp[out] Info property in Simulink bus corresponding to variable-length array
 * @param msgProp[in]      Variable-length array property in in roscpp message
 * @param warnStatus[in]   Handler for warnings during conversion
 */
template <typename BusType, typename BusInfoType, typename MsgType>
inline void convertToBusVariableNestedArray(BusType &busProp, BusInfoType &busInfoProp, 
        const MsgType &msgProp, const slros::ROSPropertyWarnStatus &warnStatus)
{       
    const int numItemsToCopy = slros::setReceivedAndCurrentLengths(
            busProp, busInfoProp, msgProp, warnStatus);    
    for (int i=0; i < numItemsToCopy; i++)
    {
        convertToBus(&busProp[i], &msgProp[i]);
    }
}


/**
 * Convert a single property (fixed-length nested array of messages) 
 * from a Simulink bus to a roscpp message.
 *
 * @param msgProp[out]    Fixed-length array property in roscpp message
 * @param busProp[in]     Fixed-length array property in Simulink bus
 */
template <typename BusType, typename MsgType>
inline void convertFromBusFixedNestedArray(MsgType &msgProp, const BusType &busProp)
{
    const int numItemsToCopy = slros::getNumItemsInFixedSimpleArray(busProp);
    for (int i=0; i < numItemsToCopy; i++)
    {
        convertFromBus(&msgProp[i], &busProp[i]);
    }        
}


/**
 * Convert a single property (variable-length nested array of messages) 
 * from a Simulink bus to a roscpp message.
 *
 * @param msgProp[out]    Variable-length array property in in roscpp message
 * @param busProp[in]     Variable-length array property in Simulink bus
 * @param busInfoProp[in] Info property in Simulink bus corresponding to variable-length array
 */
template <typename BusType, typename BusInfoType, typename MsgType>
inline void convertFromBusVariableNestedArray(MsgType &msgProp, 
        const BusType &busProp, const BusInfoType &busInfoProp) 
{
    const int numItemsToCopy = busInfoProp.CurrentLength;
    msgProp.resize(numItemsToCopy);
    for (int i=0; i < numItemsToCopy; i++)
    {
        convertFromBus(&msgProp[i], &busProp[i]);
    }     
}

//-------------------------------------------------------
// Conversion routines  for arrays of primitive values (e.g., int16,float32).
// Note: 'string', 'time', and 'duration' are handled separately during 
// roscpp <-> Simulink conversion.

/**
 * Convert a single property (fixed-length array of primitive values) 
 * from a roscpp message to Simulink bus. 
 *
 * @param busProp[out]   Fixed-length array property in Simulink bus
 * @param msgProp[in]    Fixed-length array property in roscpp message
 * @param warnStatus[in] Handler for warnings during conversion
 */
template <typename BusType, typename MsgType>
inline void convertToBusFixedPrimitiveArray(BusType &busProp, const MsgType &msgProp, 
        const slros::ROSPropertyWarnStatus &warnStatus)
{    
    const int numItemsToCopy = slros::getNumItemsInFixedSimpleArray(busProp);
    std::copy(msgProp.begin(), msgProp.begin() + numItemsToCopy, busProp);
}


/**
 * Convert a single property (variable-length array of primitive values) 
 * from a roscpp message to Simulink bus. When the roscpp array is shorter 
 * than the maximum length of the Simulink array, the remaining elements 
 * in the Simulink array left untouched (i.e., they are not explicitly zeroed
 * out). 
 * 
 * @param busProp[out]     Variable-length array property in Simulink bus
 * @param busInfoProp[out] Info property in Simulink bus corresponding to variable-length array
 * @param msgProp[in]      Variable-length array property in in roscpp message
 * @param warnStatus[in]   Handler for warnings during conversion
 */
template <typename BusType, typename BusInfoType, typename MsgType>
inline void convertToBusVariablePrimitiveArray(BusType &busProp, BusInfoType &busInfoProp, 
        const MsgType &msgProp, const slros::ROSPropertyWarnStatus &warnStatus)
{        
    const int numItemsToCopy = slros::setReceivedAndCurrentLengths(
            busProp, busInfoProp, msgProp, warnStatus);
    std::copy(msgProp.begin(), msgProp.begin() + numItemsToCopy, busProp);
}


/**
 * Convert a single property (fixed-length array of primitive values)
 * from a Simulink bus to a roscpp message.
 *
 * @param msgProp[out]    Fixed-length array property in roscpp message
 * @param busProp[in]     Fixed-length array property in Simulink bus
 */
template <typename BusType, typename MsgType>
inline void convertFromBusFixedPrimitiveArray(MsgType &msgProp, const BusType &busProp)
{
    const int numItemsToCopy = slros::getNumItemsInFixedSimpleArray(busProp);
    std::copy(busProp, busProp + numItemsToCopy, msgProp.begin());
}


/**
 * Convert a single property (variable-length array of primitive values)
 * from a Simulink bus to a roscpp message.
 *
 * @param msgProp[out]    Variable-length array property in roscpp message
 * @param busProp[in]     Variable-length array property in Simulink bus
 * @param busInfoProp[in] Info property in Simulink bus corresponding to variable-length array 
 */
template <typename BusType, typename BusInfoType, typename MsgType>
inline void convertFromBusVariablePrimitiveArray(MsgType &msgProp, 
        const BusType &busProp, const BusInfoType &busInfoProp) 
{
    const int numItemsToCopy = busInfoProp.CurrentLength;
    msgProp.resize(numItemsToCopy);
    std::copy(busProp, busProp + numItemsToCopy, msgProp.begin());
}


//-------------------------------------------------------
// Conversion routines  for string arrays
// Note: string arrays in roscpp are represented as nested arrays of std_msgs/String in Simulink


/**
 * Convert a single property (fixed-length array of strings) 
 * from a roscpp message to Simulink bus. 
 *
 * @param busProp[out]   Fixed-length string array property in Simulink bus
 * @param msgProp[in]    Fixed-length string array property in roscpp message
 * @param warnStatus[in] Handler for warnings during conversion
 */
template <typename BusType, typename MsgType>
inline void convertToBusFixedStringArray(BusType &busProp, const MsgType &msgProp, 
        const slros::ROSPropertyWarnStatus &warnStatus)
{  
    const int numStringsToCopy = slros::getNumItemsInFixedSimpleArray(busProp);
    slros::convertToBusStringsInStringArray(busProp, msgProp, warnStatus, numStringsToCopy);
}


/**
 * Convert a single property (variable-length array of strings) 
 * from a roscpp message to Simulink bus. When the roscpp array is shorter 
 * than the maximum length of the Simulink array, the remaining elements 
 * in the Simulink array left untouched (i.e., they are not explicitly zeroed
 * out).
 *
 * @param busProp[out]     Variable-length string array property in Simulink bus
 * @param busInfoProp[out] Info property in Simulink bus corresponding to variable-length string array
 * @param msgProp[in]      Variable-length string array property in roscpp message
 * @param warnStatus[in]   Handler for warnings during conversion
 */
template <typename BusType, typename BusInfoType, typename MsgType>
inline void convertToBusVariableStringArray(BusType &busProp, BusInfoType &busInfoProp, 
        const MsgType &msgProp, const slros::ROSPropertyWarnStatus &warnStatus)
{
    const int numStringsToCopy = slros::setReceivedAndCurrentLengths(
            busProp, busInfoProp, msgProp, warnStatus);    
    slros::convertToBusStringsInStringArray(busProp, msgProp, warnStatus, numStringsToCopy);
}


/**
 * Convert a single property (fixed-length array of strings) 
 * from a Simulink bus to a roscpp messsage.
 *
 * @param msgProp[out]    Fixed-length string array property in roscpp message
 * @param busProp[in]     Fixed-length string array property in Simulink bus
 */
template <typename BusType, typename MsgType>
inline void convertFromBusFixedStringArray(MsgType &msgProp, const BusType &busProp) 
{
    const int numStringsToCopy = slros::getNumItemsInFixedSimpleArray(busProp);
    slros::convertFromBusStringsInStringArray(msgProp, busProp, numStringsToCopy);
}


/**
 * Convert a single property (variable-length array of strings) 
 * from Simulink bus to a roscpp message.
 *
 * @param msgProp[out]    Fixed-length string array property in roscpp message
 * @param busProp[in]     Fixed-length string array property in Simulink bus
 * @param busInfoProp[in] Info property in Simulink bus corresponding to variable-length string array
 */
template <typename BusType, typename BusInfoType, typename MsgType>
inline void convertFromBusVariableStringArray(MsgType &msgProp, 
        const BusType &busProp, const BusInfoType &busInfoProp) 
{
    const int numStringsToCopy = busInfoProp.CurrentLength;
    msgProp.resize(numStringsToCopy);
    slros::convertFromBusStringsInStringArray(msgProp, busProp, numStringsToCopy);
}

#endif
