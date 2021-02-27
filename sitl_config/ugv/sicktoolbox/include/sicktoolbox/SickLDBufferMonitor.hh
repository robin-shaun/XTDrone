/*!
 * \file SickLDBufferMonitor.hh
 * \brief Defines a class for monitoring the receive
 *        buffer when interfacing w/ a Sick LMS LIDAR.
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

#ifndef SICK_LD_BUFFER_MONITOR_HH
#define SICK_LD_BUFFER_MONITOR_HH

#define DEFAULT_SICK_BYTE_TIMEOUT         (35000)  ///< Max allowable time between consecutive bytes

/* Definition dependencies */
#include "SickLDMessage.hh"
#include "SickBufferMonitor.hh"
#include "SickException.hh"

/* Associate the namespace */
namespace SickToolbox {

  /*!
   * \brief A class for monitoring the receive buffer when interfacing with a Sick LD LIDAR
   */
  class SickLDBufferMonitor : public SickBufferMonitor< SickLDBufferMonitor, SickLDMessage > {

  public:

    /** A standard constructor */
    SickLDBufferMonitor( );

    /** A method for extracting a single message from the stream */
    void GetNextMessageFromDataStream( SickLDMessage &sick_message ) throw( SickIOException );

    /** A standard destructor */
    ~SickLDBufferMonitor( );

  };
    
} /* namespace SickToolbox */

#endif /* SICK_LD_BUFFER_MONITOR_HH */
