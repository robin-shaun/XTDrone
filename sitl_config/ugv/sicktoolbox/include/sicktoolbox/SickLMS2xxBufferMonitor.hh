/*!
 * \file SickLMS2xxBufferMonitor.hh
 * \brief Defines a class for monitoring the receive
 *        buffer when interfacing w/ a Sick LMS 2xx
 *        laser range finder.
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

#ifndef SICK_LMS_2XX_BUFFER_MONITOR_HH
#define SICK_LMS_2XX_BUFFER_MONITOR_HH

#define DEFAULT_SICK_LMS_2XX_SICK_BYTE_TIMEOUT      (35000)  ///< Max allowable time between consecutive bytes

/* Definition dependencies */
#include "SickLMS2xxMessage.hh"
#include "SickBufferMonitor.hh"
#include "SickException.hh"

/* Associate the namespace */
namespace SickToolbox {

  /*!
   * \brief A class for monitoring the receive buffer when interfacing with a Sick LMS LIDAR
   */
  class SickLMS2xxBufferMonitor : public SickBufferMonitor< SickLMS2xxBufferMonitor, SickLMS2xxMessage > {

  public:

    /** A standard constructor */
    SickLMS2xxBufferMonitor( );

    /** A method for extracting a single message from the stream */
    void GetNextMessageFromDataStream( SickLMS2xxMessage &sick_message ) throw( SickIOException );

    /** A standard destructor */
    ~SickLMS2xxBufferMonitor( );

  };
    
} /* namespace SickToolbox */

#endif /* SICK_LMS_2XX_BUFFER_MONITOR_HH */
