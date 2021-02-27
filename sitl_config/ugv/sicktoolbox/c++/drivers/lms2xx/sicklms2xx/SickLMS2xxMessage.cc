/*!
 * \file SickLMS2xxMessage.cc
 * \brief Implementation of class SickFrame.
 * 
 * Code by Jason C. Derenick and Thomas H. Miller.
 * Contact derenick(at)lehigh(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2008, Jason C. Derenick and Thomas H. Miller
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

/* Auto-generated header */
#include <sicktoolbox/SickConfig.hh>

/* Implementation dependencies */
#include <iomanip>
#include <iostream>

#include <sicktoolbox/SickLMS2xxMessage.hh>
#include <sicktoolbox/SickLMS2xxUtility.hh>

/* Associate the namespace */
namespace SickToolbox {

  /*!
   * \brief A default constructor
   */
  SickLMS2xxMessage::SickLMS2xxMessage( ) {

    /* Initialize the object */
    Clear(); 
  }

  /*!
   * \brief A constructor for building a message from the given parameter values
   * \param dest_address The source address of the message
   * \param payload_buffer The payload of the message as an array of bytes (including the command code)
   * \param payload_length The length of the payload array in bytes
   */
  SickLMS2xxMessage::SickLMS2xxMessage( const uint8_t dest_address, const uint8_t * const payload_buffer, const unsigned int payload_length ) :
    SickMessage< SICK_LMS_2XX_MSG_HEADER_LEN, SICK_LMS_2XX_MSG_PAYLOAD_MAX_LEN, SICK_LMS_2XX_MSG_TRAILER_LEN >()  {

    /* Build the message */
    BuildMessage(dest_address,payload_buffer,payload_length);
  }

  /*!
   * \brief A constructor for parsing a byte sequence into a message object
   * \param message_buffer A well-formed message to be parsed into the class' fields
   */
  SickLMS2xxMessage::SickLMS2xxMessage( uint8_t * const message_buffer ) :
    SickMessage< SICK_LMS_2XX_MSG_HEADER_LEN, SICK_LMS_2XX_MSG_PAYLOAD_MAX_LEN, SICK_LMS_2XX_MSG_TRAILER_LEN >()  {

    /* Parse the byte sequence into a message object */
    ParseMessage(message_buffer);
  }

  /*!
   * \brief Consructs a message object from given parameter values
   * \param dest_address The destination address of the frame
   * \param *payload_buffer The payload for the frame as an array of bytes (including the command code)
   * \param payload_length The length of the payload array in bytes
   */
  void SickLMS2xxMessage::BuildMessage( const uint8_t dest_address, const uint8_t * const payload_buffer,
				     const unsigned int payload_length ) {

    /* Call the parent method!
     * NOTE: The parent method resets the object and assigns _message_length, _payload_length,
     *       _populated and copies the payload into the message buffer at the correct position
     */
    SickMessage< SICK_LMS_2XX_MSG_HEADER_LEN, SICK_LMS_2XX_MSG_PAYLOAD_MAX_LEN, SICK_LMS_2XX_MSG_TRAILER_LEN >
      ::BuildMessage(payload_buffer,payload_length);

    /*
     * Set the message header!
     */
    _message_buffer[0] = 0x02;         // Start of transmission (stx)
    _message_buffer[1] = dest_address; // Sick LMS address
        
    /* Include the payload length in the header */
    uint16_t payload_length_16 = host_to_sick_lms_2xx_byte_order((uint16_t)_payload_length);
    memcpy(&_message_buffer[2],&payload_length_16,2);

    /*
     * Set the message trailer!
     */    
    _checksum = host_to_sick_lms_2xx_byte_order(_computeCRC(_message_buffer,_payload_length+4));
    memcpy(&_message_buffer[_payload_length+4],&_checksum,2);

  }

  /*!
   * \brief Parses a sequence of bytes into a well-formed message
   * \param *message_buffer The buffer containing the source message
   */
  void SickLMS2xxMessage::ParseMessage( const uint8_t * const message_buffer ) {

    /* Call the parent method!
     * NOTE: This method resets the object and assigns _populated as true
     */
    SickMessage< SICK_LMS_2XX_MSG_HEADER_LEN, SICK_LMS_2XX_MSG_PAYLOAD_MAX_LEN, SICK_LMS_2XX_MSG_TRAILER_LEN >
      ::ParseMessage(message_buffer);
    
    /* Extract the payload length */
    uint16_t payload_length_16 = 0;
    memcpy(&payload_length_16,&message_buffer[2],2);
    _payload_length = (unsigned int)sick_lms_2xx_to_host_byte_order(payload_length_16);

    /* Compute the total message length */    
    _message_length = MESSAGE_HEADER_LENGTH + MESSAGE_TRAILER_LENGTH + _payload_length;

    /* Copy the give message into the buffer */
    memcpy(_message_buffer, message_buffer,_message_length);

    /* Extract the checksum from the frame */
    memcpy(&_checksum,&_message_buffer[_payload_length+MESSAGE_HEADER_LENGTH],2);
    _checksum = sick_lms_2xx_to_host_byte_order(_checksum);

  }

  /*!
   * \brief Reset all internal fields and buffers associated with the object.
   */
  void SickLMS2xxMessage::Clear( ) {    

    /* Call the parent method and clear out class' protected members */
    SickMessage< SICK_LMS_2XX_MSG_HEADER_LEN, SICK_LMS_2XX_MSG_PAYLOAD_MAX_LEN, SICK_LMS_2XX_MSG_TRAILER_LEN >::Clear();

    /* Reset the class' additional fields */
    _checksum = 0;
    
  }
  
  /*!
   * \brief Print the message contents.
   */
  void SickLMS2xxMessage::Print( ) const {
    
    std::cout.setf(std::ios::hex,std::ios::basefield);
    std::cout << "Checksum: " << (unsigned int) GetChecksum() << std::endl;  
    std::cout << "Dest. Addr.: " << (unsigned int) GetDestAddress() << std::endl;
    std::cout << "Command Code: " << (unsigned int) GetCommandCode() << std::endl;
    std::cout << std::flush;

    /* Call parent's print function */
    SickMessage< SICK_LMS_2XX_MSG_HEADER_LEN, SICK_LMS_2XX_MSG_PAYLOAD_MAX_LEN, SICK_LMS_2XX_MSG_TRAILER_LEN >::Print();    
  }
  
  /*!
   * \brief Computes the CRC16 of the given data buffer
   * \param data An array of bytes whose checksum to compute
   * \param len The length of the data array
   * \return CRC16 computed over given data buffer
   */
  uint16_t SickLMS2xxMessage::_computeCRC( uint8_t * data, unsigned int data_length ) const {
  
    uint16_t uCrc16;
    uint8_t abData[2];
    uCrc16 = abData[0] = 0;
    while (data_length-- ) {
      abData[1] = abData[0];
      abData[0] = *data++;
      if(uCrc16 & 0x8000) {
	uCrc16 = (uCrc16 & 0x7fff) << 1;
	uCrc16 ^= CRC16_GEN_POL;
      }
      else {
	uCrc16 <<= 1;
      }
      uCrc16 ^= MKSHORT(abData[0],abData[1]);
    }
    return uCrc16;
  }

  SickLMS2xxMessage::~SickLMS2xxMessage( ) { }

} /* namespace SickToolbox */
