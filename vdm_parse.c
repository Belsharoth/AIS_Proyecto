/* -----------------------------------------------------------------------
   libuais VDM/VDO sentence parser
   Copyright 2006-2008 by Brian C. Lane <bcl@brianlane.com>
   All rights Reserved
   ----------------------------------------------------------------------- */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "portable.h"
#include "nmea.h"
#include "sixbit.h"
#include "vdm_parse.h"

/*! \file
    \brief AIVDM/AIVDO AIS Sentence Parser
    \author Copyright 2006-2008 by Brian C. Lane <bcl@brianlane.com>, All Rights Reserved
    \version 1.8
    
    This module contains the functions to parse incoming Automatic
    Identification System (AIS) serial messages as defined by IEC-61993-2 
    Each message type has its own structure, of the form aismsg_XX, based 
    on the message id of the message. The parse routine for each message
    is of the form parse_ais_<msgid>, eg, parse_ais_5()

    Each of the aismsg_XX parsing functions expect to be passed a pointer
    to a ais_sate structure containing the raw 6-bit data and a pointer 
    to the aismsg_XX structure to place the parsed information into.

    All aismsg_XX structures are cleared by their respecive parsers, except
    for parse_ais_24() which stores 2 messages into a single structure.
 
    Data like the ship's name and callsign are converted to ASCII
    before being stored. Positions are converted to signed long values,
    and other fields like course over ground, speed, heading, etc. 
    are left for further decoding. 1/1000 minute positions are converted
    to 1/10000 minute positions to make comparisons easier.

    Not all of these parsers have been fully tested with live data. 
    Here is a list of the ones that have been tested so far:
      1,3,4,5,7,12,15,20

    If you have valid data for other message ids and would like to
    contribute them please email them to bcl@brianlane.com

    Example:
    \code
    ais_state ais;
    aismsg_1  msg_1;
    char buf[256];

    memset( &state, 0, sizeof(state) );

    // Process incoming packets from some source
    while( message_source_active )
    {
        // Get your messages from your source, whatever that may be
        fill_buf( buf, 255);

        // Is there a complete message available?
        if (assemble_vdm( &ais, buf ) == 0)
        {
            // Get the 6 bit message id
            ais.msgid = (unsigned char) get_6bit( &ais.six_state, 6 );
            
            // process message with appropriate parser
            switch( ais.msgid ) {
                case 1:
                    if( parse_ais_1( &ais, &msg_1 ) == 0 )
                    {
                        // Successfully parsed a message 1 into msg_1 struct
                    }
                    break;
            }
        }
    }
    \endcode    


*/

/*! \mainpage AIVDM/AIVDO AIS Message Parser
    <center>Copyright 2006-2008 by Brian C. Lane <bcl@brianlane.com><br>
    http://www.aisparser.com/
    </center>
    
    
    The Automatic Identification System (AIS) allows ships to be tracked in
    realtime based on information transmitted by each ship. They are equipped
    with either a Class A or Class B AIS transponder which includes a GPS 
    for accurate position and speed reporting. AIS receivers decode the
    transmitted information and output the data as AIVDM messages.
    
    The structure of the AIVDM message is described in IEC 61993-2 and it is
    a variation of the NMEA 0183 sentence format that includes the raw data
    encoded in a 6-bit format. A Message 1 example looks like this:
    
    \code
    !AIVDM,1,1,,B,19NS7Sp02wo?HETKA2K6mUM20<L=,0*27
    \endcode
    
    The meaning of each data element in the AIS messages is covered by the 
    ITU M.1371 and IEC 62287 documents. These documents are available from
    the websites of these organizations:
      - http://www.itu.int
      - http://www.iec.ch
      - http://www.navcenter.org/marcomms/IEC/
      
    To fully understand the data produced by this library you should read 
    the above documents. 
    
    This library allows you to write programs that accept ASCII AIVDM or
    AIVDO messages and parse their packed 6-bit data into structures 
    containging the message data. The general flow of a typical aisparser
    program would look like this:
    
     - Receive ASCII data (via serial port, file or network socket)
     - Pass the data to assemble_vdm() to reassemble multipart messages and
       extract their 6-bit data.
     - Fetch the first 6 bits from the data stream, this is the message id
       and determines which parser function to call and what structure
       to store the results in.
     - Call the appropriate parse_ais_XX function with aismsg_XX structure
       passed to it.
     - Act on the message data: display a position, ship name, etc.
     
    This library is thread safe, all variables used are contained in the
    ais_state structure. To support multiple data streams all you need to
    do is used 2 different instances of ais_state.

    To get started you should read the documentation on the vdm_parse.c
    module and look at the example code in the example directory of the
    source distribution. 
    
    Not all of the AIS fields are fully parsed. My philosophy is to get the
    raw data into a format that your code can easily deal with and to 
    minimize the layers between your code and the raw data. This means that
    you will need a copy of the documents mentioned above to fully understand
    the results you will get from parsing messages.
    
    eg. The sog field of a message 1,2,3 represents Speed Over Ground in 
    1/10th of a knot. In the message 1 structure it is stored as an int
    instead of dividing it by 10 and converting to a float. This allows you
    to do your own division of the raw value and minimizes potential errors.
   
*/


/* ----------------------------------------------------------------------- */
/** Get 20 bit ETA, Seaway and IMO Timetag

    \param state pointer to parser state
    \param datetime pointer to timetag result struct   

    return 
      - return 0 if there was no error
      - return 1 if there was an error
      - return 2 if there are < 20 bits to parse
*/
/* ----------------------------------------------------------------------- */
int __stdcall get_timetag( sixbit *state, timetag *datetime )
{
	int	length;
	
    if ( !state )
        return 1;
    if ( !datetime )
        return 1;

	length = sixbit_length(state);
    if ( length < 20 )
        return 2;

	datetime->month   = (char) get_6bit( state, 4 );
	datetime->day     = (char) get_6bit( state, 5 );
	datetime->hours   = (char) get_6bit( state, 5 );
	datetime->minutes = (char) get_6bit( state, 6 );
	
	return 0;
}


/* ----------------------------------------------------------------------- */
/** Convert a 2's complement value to a signed integer value

	@param sign_bit
	@param value pointer to value
	
	eg. to convert a 10 bit 2's complement:
		conv_sign( 0x00200, &temp );
*/
/* ----------------------------------------------------------------------- */
void __stdcall conv_sign( unsigned int sign_bit, int *value )
{
    if( *value & sign_bit )
    {
        *value = (sign_bit << 1) - *value;
        *value *= -1;
    }
}


/* ----------------------------------------------------------------------- */
/** Convert a AIS 6-bit character to ASCII

    \param value 6-bit value to be converted
   
    return 
      - corresponding ASCII value (0x20-0x5F)

    This function is used to convert binary data to ASCII. This is 
    different from the 6-bit ASCII to binary conversion for VDM 
    messages; it is used for strings within the datastream itself.
    eg. Ship Name and Destination.
*/
/* ----------------------------------------------------------------------- */
char __stdcall ais2ascii( char value )
{
    value = value & 0x3F;
    if( value < 0x20 )
        return value + 0x40;
    else
        return value;
}


/* ----------------------------------------------------------------------- */
/** Convert 1/10000 minute position to signed DD.DDDDDD format

    \param latitude signed latitude in 1/10000 minute format
    \param longitude signed longitude in 1/10000 minute format
    \param lat_dd pointer to hold signed latitude in DD.DDDDDD format
    \param long_ddd pointer to hold signed longitude in DD.DDDDDD format

    This function converts 1/10000 minute formatted positions to standard
    Degrees and decimal degrees format.
*/
/* ----------------------------------------------------------------------- */
int __stdcall pos2ddd( long latitude, long longitude, double *lat_dd, double *long_ddd )
{
    /* Convert 1/10000 Minute Latitude to DD.DDDDDD */
    *lat_dd = (int) (latitude / 600000);
    *lat_dd += (double) (latitude - (*lat_dd * 600000)) / 600000.0;

    /* Convert 1/10000 Minute Longitude to DDD.DDDDDD */
    *long_ddd = (int) (longitude / 600000);
    *long_ddd += (double) (longitude - (*long_ddd * 600000)) / 600000.0;

    return 0;
}

// Función inversa a la anterior //
int ddd2pos(double lat_dd, double long_ddd, long *latitude, long *longitude)
{
    *latitude = (long) (lat_dd * 600000);
    *longitude = (long) (long_ddd * 600000);
    return 0;
}

/* ----------------------------------------------------------------------- */
/** Convert 1/10000 minute position to signed DD MM.MMMM format

    \param latitude signed latitude in 1/10000 minute format
    \param longitude signed longitude in 1/10000 minute format
    \param lat_dd pointer to hold signed latitude in DD format
    \param lat_min pointer to hold minutes in mm.mmmm format
    \param long_ddd pointer to hold signed longitude in DDD format
    \param long_min pointer to hold minutes in mm.mmmm format
    

    This function converts 1/10000 minute formatted positions to standard
    Degrees and decimal minutes format.
*/
/* ----------------------------------------------------------------------- */
int __stdcall pos2dmm( long latitude, long longitude,
             short *lat_dd, double *lat_min,
             short *long_ddd, double *long_min )
{
    /* Convert 1/10000 minute Latitude to DD MM.MMMM format */
    *lat_dd = (short) (latitude / 600000);
    *lat_min = abs((latitude - (*lat_dd * 600000))) / 10000.0;

    /* Convert 1/10000 minute Longitude to DDD MM.MMMM format */
    *long_ddd = (short) (longitude / 600000);
    *long_min = abs((longitude - (*long_ddd * 600000))) / 10000.0;

    return 0;
}

/* ----------------------------------------------------------------------- */
/** Convert unsigned 1/10000 minute position to signed

    \param latitude pointer to an unsigned latitude
    \param longitude pointer to an unsigned longitude

    This function converts the raw 2's complement values to signed long
    1/10000 minute position. It is used internally by all of the messages
    that contain position information.
*/
/* ----------------------------------------------------------------------- */
int __stdcall conv_pos( long *latitude, long *longitude )
{
    /* LATITUDE */
    /* Convert latitude to signed number */
    if( *latitude & 0x4000000 )
    {
        *latitude = 0x8000000 - *latitude;
        *latitude *= -1;
    }

    /* LONGITUDE */
    /* Convert longitude to signed number */
    if( *longitude & 0x8000000 )
    {
        *longitude = 0x10000000 - *longitude;
        *longitude *= -1;
    }

    return 0;
}
// Función inversa a la anterior //
int inv_conv_pos(long *latitude, long *longitude)
{
	if (*latitude < 0)
	{
		*latitude = 0x8000000 + *latitude;
	}

	if (*longitude < 0)
	{
		*longitude = 0x10000000 + *longitude;
	}

	return 0;
}

/* ----------------------------------------------------------------------- */
/** Assemble AIVDM/VDO sentences

    This function handles re-assembly and extraction of the 6-bit data
    from AIVDM/AIVDO sentences.
    
    Because the NMEA standard limits the length of a line to 80 characters
    some AIS messages, such as message 5, are output as a multipart VDM 
    messages. 
    This routine collects the 6-bit encoded data from these parts and 
    returns a 0 when all pieces have been reassembled.
    
    It expects the sentences to:
      - Be in order, part 1, part 2, etc.
      - Be from a single sequence
      
    It will return an error if it receives a piece out of order or from
    a new sequence before the previous one is finished.
    
        
    Returns
        - 0 Complete packet
        - 1 Incomplete packet
        - 2 NMEA 0183 checksum failed
        - 3 Not an AIS message
        - 4 Error with nmea_next_field
        - 5 Out of sequence packet

    Example:
    \code
    ais_state state;
    char buf[3][255] = { "!AIVDM,1,1,,B,19NS7Sp02wo?HETKA2K6mUM20<L=,0*27\r\n",
                         "!AIVDM,2,1,9,A,55Mf@6P00001MUS;7GQL4hh61L4hh6222222220t41H,0*49\r\n",
                         "!AIVDM,2,2,9,A,==40HtI4i@E531H1QDTVH51DSCS0,2*16\r\n"
                       };
    
    memset( &state, 0, sizeof(state) );
    assemble_vdm( &state, buf[0] )
    state.six_state now has the 6-bit data from the message

    assemble_vdm( &state, buf[1] );
    This returns a 1 because not all the pieces have been received.
    
    assemble_vdm( &state, buf[2] );
    This returns a 0 and state.six_state has the complete 6-bit data for
    the message 5.
    
    \endcode

*/
/* ----------------------------------------------------------------------- */
int __stdcall assemble_vdm( ais_state *state, char *str )
{
    unsigned int  total;
    unsigned int  num;
    unsigned int  sequence;
    unsigned int  *fields[3] = { &total, &num, &sequence };
    char *p;
    char *d;
    unsigned char checksum;
    unsigned int  i;
    unsigned int length;


    /* Is the string an AIS message? Allow any start character and any
       device pair.
    */
    if ( (p = find_nmea_start( str )) == NULL )
        return 3;
    if (   (strncmp( p+3, "VDM", 3 ) != 0)
        && (strncmp( p+3, "VDO", 3 ) != 0) )
    {
        /* Not an AIS message */
        return 3;
    }

    /* Check the string's checksum */
    if (check_nmea_checksum( p, &checksum ) != 0 )
    {
        /* Checksum failed */
        return 2;
    }

    /* Get the 3 message info values from the packet */
    for (i=0; i<3; i++)
    {
        if ( (p = nmea_next_field( p )) == NULL )
        {
            /* Error with the string */
            return 4;
        }
        /* Get an integer from the string */
        *fields[i] = nmea_uint(p);
    }
    
    /* Are we looking for more parts? */
    if (state->total > 0)
    {
        /* If the sequence doesn't match, or the number is not in 
           order: reset and exit
        */
        if( (state->sequence != sequence) || (state->num != num-1) )
        {
            state->total = 0;
            state->sequence =0;
            state->num = 0;
            return 5;
        }
        state->num++;
        
    } else {
        /* Not looking for more parts, reset the state */
        state->total = total;
        state->num = num;
        state->sequence = sequence;
        init_6bit( &state->six_state );
    }
    
    /* Get the channel character */
    if ( (p = nmea_next_field( p )) == NULL )
    {
        /* Error with the string */
        return 4;
    }
	
    state->channel = *p;
    
    /* Point to the 6-bit data */
    if ( (p = nmea_next_field( p )) == NULL )
    {
        /* Error with the string */
        return 4;
    }

    /* Copy the 6-bit ASCII field over to the sixbit_state buffer */
    d = state->six_state.bits;
    length = 0;
    /* Find the end of the current string and count the number of bytes */
    while( d && (*d != 0))
    {
        length++;
        d++;
    }
    nmea_copy_field( d, p, SIXBIT_LEN - length );

    /* Is this the last part of the sequence? */
    if ((total==0) || (state->total == num))
    {
        state->total    = 0;
        state->num      = 0;
        state->sequence = 0;

        /* Found a complete packet */
        return 0;
    }
    
    return 1;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 1 into an aismsg_1 structure

    Ship Position report with SOTDMA Communication state

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return:
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error

    Note that the latitude and longitude are converted to signed values
    before being returned.
*/
/* ----------------------------------------------------------------------- */
int __stdcall parse_ais_1( ais_state *state, aismsg_1 *result )
{
    if( !state )
        return 1;
    if( !result )
        return 1;

    if (strlen(state->six_state.bits) != 28 )
    {
        return 2;
    }

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_1 ) );

    result->msgid = state->msgid;

    /* Parse the Message 1 */
    result->repeat       = (char)          get_6bit( &state->six_state, 2  );
    result->userid       = (unsigned long) get_6bit( &state->six_state, 30 );
    result->nav_status   = (char)          get_6bit( &state->six_state, 4  );
    result->rot          = (unsigned char) get_6bit( &state->six_state, 8  );
    result->sog          = (int)           get_6bit( &state->six_state, 10 );
    result->pos_acc      = (char)          get_6bit( &state->six_state, 1  );
    result->longitude    = (long)          get_6bit( &state->six_state, 28 );
    result->latitude     = (long)          get_6bit( &state->six_state, 27 );
    result->cog          = (int)           get_6bit( &state->six_state, 12 );
    result->true         = (int)           get_6bit( &state->six_state, 9  );
    result->utc_sec      = (char)          get_6bit( &state->six_state, 6  );
    result->regional     = (char)          get_6bit( &state->six_state, 4  );
    result->spare        = (char)          get_6bit( &state->six_state, 1  );
    result->raim         = (char)          get_6bit( &state->six_state, 1  );
    result->sync_state   = (char)          get_6bit( &state->six_state, 2  );
    result->slot_timeout = (char)          get_6bit( &state->six_state, 3  );
    result->sub_message  = (int)           get_6bit( &state->six_state, 14 );

    /* Convert the position to signed value */
    //conv_pos( &result->latitude, &result->longitude);

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 2 into an aismsg_2 structure

   Ship Position report with SOTDMA Communication state

   \param state    pointer to ais_state
   \param result   pointer to parsed message result structure to be filled
   
   return:
     - 0 if no errors
     - 1 if there is an error
     - 2 if there is a packet length error

    Note that the latitude and longitude are converted to signed values
    before being returned.
*/
/* ----------------------------------------------------------------------- */
int __stdcall parse_ais_2( ais_state *state, aismsg_2 *result )
{
    int length;

    if( !state )
        return 1;
    if( !result )
        return 1;
    
    length = strlen(state->six_state.bits) * 6;
    if( length != 168 )
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_2 ));

    result->msgid = state->msgid;

    /* Parse the Message 2 */
    result->repeat       = (char)          get_6bit( &state->six_state, 2  );
    result->userid       = (unsigned long) get_6bit( &state->six_state, 30 );
    result->nav_status   = (char)          get_6bit( &state->six_state, 4  );
    result->rot          = (unsigned char) get_6bit( &state->six_state, 8  );
    result->sog          = (int)           get_6bit( &state->six_state, 10 );
    result->pos_acc      = (char)          get_6bit( &state->six_state, 1  );
    result->longitude    = (long)          get_6bit( &state->six_state, 28 );
    result->latitude     = (long)          get_6bit( &state->six_state, 27 );
    result->cog          = (int)           get_6bit( &state->six_state, 12 );
    result->true         = (int)           get_6bit( &state->six_state, 9  );
    result->utc_sec      = (char)          get_6bit( &state->six_state, 6  );
    result->regional     = (char)          get_6bit( &state->six_state, 4  );
    result->spare        = (char)          get_6bit( &state->six_state, 1  );
    result->raim         = (char)          get_6bit( &state->six_state, 1  );
    result->sync_state   = (char)          get_6bit( &state->six_state, 2  );
    result->slot_timeout = (char)          get_6bit( &state->six_state, 3  );
    result->sub_message  = (int)           get_6bit( &state->six_state, 14 );

    /* Convert the position to signed value */
    conv_pos( &result->latitude, &result->longitude);

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 3 into an aismsg_3 structure

   Ship Position report with ITDMA Communication state

   \param state    pointer to ais_state
   \param result   pointer to parsed message result structure to be filled
   
   return:
     - 0 if no errors
     - 1 if there is an error
     - 2 if there is a packet length error

    Note that the latitude and longitude are converted to signed values
    before being returned.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_3( ais_state *state, aismsg_3 *result )
{
    if( !state )
        return 1;
    if( !result )
        return 1;

    if (strlen(state->six_state.bits) != 28 )
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_3 ));

    result->msgid = state->msgid;

    /* Parse the Message 3 */
    result->repeat         = (char)          get_6bit( &state->six_state, 2  );
    result->userid         = (unsigned long) get_6bit( &state->six_state, 30 );
    result->nav_status     = (char)          get_6bit( &state->six_state, 4  );
    result->rot            = (unsigned char) get_6bit( &state->six_state, 8  );
    result->sog            = (int)           get_6bit( &state->six_state, 10 );
    result->pos_acc        = (char)          get_6bit( &state->six_state, 1  );
    result->longitude      = (long)          get_6bit( &state->six_state, 28 );
    result->latitude       = (long)          get_6bit( &state->six_state, 27 );
    result->cog            = (int)           get_6bit( &state->six_state, 12 );
    result->true           = (int)           get_6bit( &state->six_state, 9  );
    result->utc_sec        = (char)          get_6bit( &state->six_state, 6  );
    result->regional       = (char)          get_6bit( &state->six_state, 4  );
    result->spare          = (char)          get_6bit( &state->six_state, 1  );
    result->raim           = (char)          get_6bit( &state->six_state, 1  );
    result->sync_state     = (char)          get_6bit( &state->six_state, 2  );
    result->slot_increment = (int)           get_6bit( &state->six_state, 13 );
    result->num_slots      = (char)          get_6bit( &state->six_state, 3  );
    result->keep           = (char)          get_6bit( &state->six_state, 1  );

    /* Convert the position to signed value */
    conv_pos( &result->latitude, &result->longitude);

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 4 into an aismsg_4 structure

    Base Station Report

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return:
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error

     Note that the latitude and longitude are converted to signed values
     before being returned.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_4( ais_state *state, aismsg_4 *result )
{
    if( !state )
        return 1;
    if( !result )
        return 1;

    if (strlen(state->six_state.bits) != 28)
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_4 ));

    result->msgid = state->msgid;

    /* Parse the Message 4 */
    result->repeat       = (char)           get_6bit( &state->six_state, 2  );
    result->userid       = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->utc_year     = (int)            get_6bit( &state->six_state, 14 );
    result->utc_month    = (char)           get_6bit( &state->six_state, 4  );
    result->utc_day      = (char)           get_6bit( &state->six_state, 5  );
    result->utc_hour     = (char)           get_6bit( &state->six_state, 5  );
    result->utc_minute   = (char)           get_6bit( &state->six_state, 6  );
    result->utc_second   = (char)           get_6bit( &state->six_state, 6  );
    result->pos_acc      = (char)           get_6bit( &state->six_state, 1  );
    result->longitude    = (long)           get_6bit( &state->six_state, 28 );
    result->latitude     = (long)           get_6bit( &state->six_state, 27 );
    result->pos_type     = (char)           get_6bit( &state->six_state, 4  );
    result->spare        = (int)            get_6bit( &state->six_state, 10 );
    result->raim         = (char)           get_6bit( &state->six_state, 1  );
    result->sync_state   = (char)           get_6bit( &state->six_state, 2  );
    result->slot_timeout = (char)           get_6bit( &state->six_state, 3  );
    result->sub_message  = (int)            get_6bit( &state->six_state, 14 );

    /* Convert the position to signed value */
    conv_pos( &result->latitude, &result->longitude);

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 5 into an aismsg_5 structure

    Ship Static and Voyage Data

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
     return:
       - 0 if no errors
       - 1 if there is an error
       - 2 if there is a packet length error

     Note that the ship's callsign, name and destination are converted to
     ASCII before storage. 
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_5( ais_state *state, aismsg_5 *result )
{
    unsigned int i;

    if( !state )
        return 1;
    if( !result )
        return 1;

    if (strlen(state->six_state.bits) != 71)
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_5 ));

    result->msgid = state->msgid;

    /* Parse the Message 5 */
    result->repeat       = (char)           get_6bit( &state->six_state, 2  );
    result->userid       = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->version      = (char)           get_6bit( &state->six_state, 2  );
    result->imo          = (unsigned long)  get_6bit( &state->six_state, 30 );

    /* Get the Callsign, convert to ASCII */
    i = 0;
    while( i != 7 )
    {
        result->callsign[i] = ais2ascii( (char) get_6bit( &state->six_state, 6 ));
        i++;
    }
    result->callsign[i] = 0;

    /* Get the Ship Name, convert to ASCII */
    i = 0;
    while( i != 20 )
    {
        result->name[i] = ais2ascii( (char) get_6bit( &state->six_state, 6 ));
        i++;
    }
    result->name[i] = 0;

    result->ship_type    = (unsigned char)  get_6bit( &state->six_state, 8  );
    result->dim_bow      = (int)            get_6bit( &state->six_state, 9  );
    result->dim_stern    = (int)            get_6bit( &state->six_state, 9  );
    result->dim_port     = (char)           get_6bit( &state->six_state, 6  );
    result->dim_starboard= (char)           get_6bit( &state->six_state, 6  );
    result->pos_type     = (char)           get_6bit( &state->six_state, 4  );
    result->eta          = (unsigned long)  get_6bit( &state->six_state, 20 );
    result->draught      = (unsigned char)  get_6bit( &state->six_state, 8  );

    /* Get the Ship Destination, convert to ASCII */
    i = 0;
    while( i != 20 )
    {
        result->dest[i] = ais2ascii( (char) get_6bit( &state->six_state, 6 ));
        i++;
    }
    result->dest[i] = 0;
    
    result->dte          = (char) get_6bit( &state->six_state, 1 );
    result->spare        = (char) get_6bit( &state->six_state, 1 );

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 6 into an aismsg_6 structure

    Addressed Binary Message

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return:
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error

    Note: the binary payload of the message is left in its 6-bit ASCII
          form when stored into result->data. This allows the data to
          be passed to the sixbit module for parsing.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_6( ais_state *state, aismsg_6 *result )
{
    int length;
    
    if( !state )
        return 1;
    if( !result )
        return 1;

    length = strlen(state->six_state.bits) * 6;
    if( (length < 88) || (length > 1008) )
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_6 ));

    result->msgid = state->msgid;

    /* Parse the Message 6 */
    result->repeat       = (char)           get_6bit( &state->six_state, 2  );
    result->userid       = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->sequence     = (char)           get_6bit( &state->six_state, 2 );
    result->destination  = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->retransmit   = (char)           get_6bit( &state->six_state, 1 );
    result->spare        = (char)           get_6bit( &state->six_state, 1 );
    result->app_id       = (unsigned int)   get_6bit( &state->six_state, 16 );

    /* Store the remaining payload of the packet for further processing */
	result->data = state->six_state;

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 7 into an aismsg_7 structure

    Binary acknowledge

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return:
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error

    Depending on the length of the message some of the fields may be 0.
    result->num_acks has the number of acks received.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_7( ais_state *state, aismsg_7 *result )
{
    int length;

    if( !state )
        return 1;
    if( !result )
        return 1;

    /* Check the length of the packet */
    length = strlen( state->six_state.bits ) * 6;
    if( (length < 72) || (length > 168) )
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_7 ));

    result->msgid = state->msgid;

    /* Parse the Message 7 */
    result->repeat       = (char)           get_6bit( &state->six_state, 2  );
    result->userid       = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->spare        = (char)           get_6bit( &state->six_state, 2  );
    result->destid_1     = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->sequence_1   = (char)           get_6bit( &state->six_state, 2  );
    result->num_acks = 1;

    if( length > 72 )
    {
        result->destid_2     = (unsigned long)  get_6bit( &state->six_state, 30 );
        result->sequence_2   = (char)           get_6bit( &state->six_state, 2  );
        result->num_acks++;
    }
    if( length > 104 )
    {
        result->destid_3     = (unsigned long)  get_6bit( &state->six_state, 30 );
        result->sequence_3   = (char)           get_6bit( &state->six_state, 2  );
        result->num_acks++;
    }
    if( length > 136 )
    {
        result->destid_4     = (unsigned long)  get_6bit( &state->six_state, 30 );
        result->sequence_4   = (char)           get_6bit( &state->six_state, 2  );
        result->num_acks++;
    }

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 8 into an aismsg_8 structure

   Binary Broadcast Message

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
   return:
     - 0 if no errors
     - 1 if there is an error
     - 2 if there is a packet length error

    Note: the binary payload of the message is left in its 6-bit ASCII
          form when stored into result->data. This allows the data to
          be passed to the sixbit module for parsing.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_8( ais_state *state, aismsg_8 *result )
{
    int length;

    if( !state )
        return 1;
    if( !result )
        return 1;

    length = strlen(state->six_state.bits) * 6;
    if( (length < 56) || (length > 1008) )
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_8 ));

    result->msgid = state->msgid;

    /* Parse the Message 8 */
    result->repeat       = (char)           get_6bit( &state->six_state, 2  );
    result->userid       = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->spare        = (char)           get_6bit( &state->six_state, 2 );
    result->app_id       = (unsigned int)   get_6bit( &state->six_state, 16 );

    /* Store the remaining payload of the packet for further processing */
	result->data = state->six_state;

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 9 into an aismsg_9 structure

    Search And Rescue(SAR) position report

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return:
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error

     Note that the latitude and longitude are converted to signed values
     before being returned.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_9( ais_state *state, aismsg_9 *result )
{
    if( !state )
        return 1;
    if( !result )
        return 1;

    if (strlen(state->six_state.bits) != 28)
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_9 ));

    result->msgid = state->msgid;

    /* Parse the Message 9 */
    result->repeat     = (char)           get_6bit( &state->six_state, 2  );
    result->userid     = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->altitude   = (int)            get_6bit( &state->six_state, 12 );
    result->sog        = (int)            get_6bit( &state->six_state, 10 );
    result->pos_acc    = (char)           get_6bit( &state->six_state, 1  );
    result->longitude  = (long)           get_6bit( &state->six_state, 28 );
    result->latitude   = (long)           get_6bit( &state->six_state, 27 );
    result->cog        = (int)            get_6bit( &state->six_state, 12 );
    result->utc_sec    = (char)           get_6bit( &state->six_state, 6  );
    result->regional   = (unsigned char)  get_6bit( &state->six_state, 8  );
    result->dte        = (char)           get_6bit( &state->six_state, 1  );
    result->spare      = (char)           get_6bit( &state->six_state, 3  );
    result->assigned   = (char)           get_6bit( &state->six_state, 1  );
    result->raim       = (char)           get_6bit( &state->six_state, 1  );
    result->comm_state = (char)           get_6bit( &state->six_state, 1  );

    if( result->comm_state == 0 )
    {
        result->sotdma.sync_state   = (char)  get_6bit( &state->six_state, 2  );
        result->sotdma.slot_timeout = (char)  get_6bit( &state->six_state, 3  );
        result->sotdma.sub_message  = (int)   get_6bit( &state->six_state, 14 );
    } else {
        result->itdma.sync_state    = (char)  get_6bit( &state->six_state, 2  );
        result->itdma.slot_inc      = (int)   get_6bit( &state->six_state, 13 );
        result->itdma.num_slots     = (char)  get_6bit( &state->six_state, 3  );
        result->itdma.keep_flag     = (char)  get_6bit( &state->six_state, 1  );
    }

    /* Convert the position to signed value */
    conv_pos( &result->latitude, &result->longitude);

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 10 into an aismsg_10 structure

    UTC and Date Inquiry

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return:
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_10( ais_state *state, aismsg_10 *result )
{
    if( !state )
        return 1;
    if( !result )
        return 1;

    if (strlen(state->six_state.bits) != 12)
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_10 ));

    result->msgid = state->msgid;

    /* Parse the Message 10 */
    result->repeat     = (char)           get_6bit( &state->six_state, 2  );
    result->userid     = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->spare1     = (char)           get_6bit( &state->six_state, 2  );
    result->destination= (unsigned long)  get_6bit( &state->six_state, 30 );
    result->spare2     = (char)           get_6bit( &state->six_state, 2  );

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 11 into an aismsg_11 structure

    UTC and Date Response

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return:
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_11( ais_state *state, aismsg_11 *result )
{
    if( !state )
        return 1;
    if( !result )
        return 1;

    if (strlen(state->six_state.bits) != 28)
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_11 ));

    result->msgid = state->msgid;

    /* Parse the Message 11 */
    result->repeat       = (char)           get_6bit( &state->six_state, 2  );
    result->userid       = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->utc_year     = (int)            get_6bit( &state->six_state, 14 );
    result->utc_month    = (char)           get_6bit( &state->six_state, 4  );
    result->utc_day      = (char)           get_6bit( &state->six_state, 5  );
    result->utc_hour     = (char)           get_6bit( &state->six_state, 5  );
    result->utc_minute   = (char)           get_6bit( &state->six_state, 6  );
    result->utc_second   = (char)           get_6bit( &state->six_state, 6  );
    result->pos_acc      = (char)           get_6bit( &state->six_state, 1  );
    result->longitude    = (long)           get_6bit( &state->six_state, 28 );
    result->latitude     = (long)           get_6bit( &state->six_state, 27 );
    result->pos_type     = (char)           get_6bit( &state->six_state, 4  );
    result->spare        = (int)            get_6bit( &state->six_state, 10 );
    result->raim         = (char)           get_6bit( &state->six_state, 1  );
    result->sync_state   = (char)           get_6bit( &state->six_state, 2  );
    result->slot_timeout = (char)           get_6bit( &state->six_state, 3  );
    result->sub_message  = (int)            get_6bit( &state->six_state, 14 );

    /* Convert the position to signed value */
    conv_pos( &result->latitude, &result->longitude);

    return 0;
}



/* ----------------------------------------------------------------------- */
/** Parse an AIS message 12 into an aismsg_12 structure

    Addressed Safety Related Message

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return:
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error

    The safety message is converted to ASCII before storage in the
    result->message field.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_12( ais_state *state, aismsg_12 *result )
{
    int length;
    int i;
    
    if( !state )
        return 1;
    if( !result )
        return 1;

    length = strlen(state->six_state.bits) * 6;
    if( (length < 72) || (length > 1008) )
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_12 ));

    result->msgid = state->msgid;

    /* Parse the Message 12 */
    result->repeat       = (char)           get_6bit( &state->six_state, 2  );
    result->userid       = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->sequence     = (char)           get_6bit( &state->six_state, 2  );
    result->destination  = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->retransmit   = (char)           get_6bit( &state->six_state, 1  );
    result->spare        = (char)           get_6bit( &state->six_state, 1  );

    /* Get the message, convert to ASCII */
    i = 0;
    while( i != (length - 72) / 6 )
    {
        result->message[i] = ais2ascii( (char) get_6bit( &state->six_state, 6 ));
        i++;
    }
    result->message[i] = 0;


    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 13 into an aismsg_13 structure

    Safety Related Acknowledge

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error

    Depending on the length of the message some of the fields may be 0.
    result->num_acks has the number of acks received.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_13( ais_state *state, aismsg_13 *result )
{
    int length;

    if( !state )
        return 1;
    if( !result )
        return 1;

    /* Check the length of the packet */
    length = strlen( state->six_state.bits ) * 6;
    if( (length < 72) || (length > 168) )
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_13 ));

    result->msgid = state->msgid;

    /* Parse the Message 13 */
    result->repeat       = (char)           get_6bit( &state->six_state, 2  );
    result->userid       = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->spare        = (char)           get_6bit( &state->six_state, 2  );
    result->destid_1     = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->sequence_1   = (char)           get_6bit( &state->six_state, 2  );
    result->num_acks = 1;

    if( length > 72 )
    {
        result->destid_2     = (unsigned long)  get_6bit( &state->six_state, 30 );
        result->sequence_2   = (char)           get_6bit( &state->six_state, 2  );
        result->num_acks++;
    }
    if( length > 104 )
    {
        result->destid_3     = (unsigned long)  get_6bit( &state->six_state, 30 );
        result->sequence_3   = (char)           get_6bit( &state->six_state, 2  );
        result->num_acks++;
    }
    if( length > 136 )
    {
        result->destid_4     = (unsigned long)  get_6bit( &state->six_state, 30 );
        result->sequence_4   = (char)           get_6bit( &state->six_state, 2  );
        result->num_acks++;
    }

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 14 into an aismsg_14 structure

    Safety Related Broadcast

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return:
      - 0 if no errors
      - 1 if there is a parameter error
      - 2 if there is a packet length error

    The safety message is converted to ASCII before storage in the
    result->message field.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_14( ais_state *state, aismsg_14 *result )
{
    int length;
    int i;

    if( !state )
        return 1;
    if( !result )
        return 1;

    length = strlen(state->six_state.bits) * 6;
    if( (length < 40) || (length > 1008) )
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_14 ));

    result->msgid = state->msgid;

    /* Parse the Message 14 */
    result->repeat       = (char)           get_6bit( &state->six_state, 2  );
    result->userid       = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->spare        = (char)           get_6bit( &state->six_state, 2 );

    /* Get the message, convert to ASCII */
    i = 0;
    while( i != (length - 40) / 6 )
    {
        result->message[i] = ais2ascii( (char) get_6bit( &state->six_state, 6 ));
        i++;
    }
    result->message[i] = 0;

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 15 into an aismsg_15 structure

    Interrogation

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error

    Depending on the length of the message some if the fields may be 0.
    result->num_reqs has the number of interrogation requests.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_15( ais_state *state, aismsg_15 *result )
{
    int length;

    if( !state )
        return 1;
    if( !result )
        return 1;

    /* Check the length of the packet */
    length = strlen( state->six_state.bits ) * 6;
    if( (length < 88) || (length > 162) )
        return 2;

    /* Clear it all to zeros */
    memset( result, 0, sizeof( aismsg_15 ));

    result->msgid = state->msgid;

    /* Parse the Message 15 */
    result->repeat       = (char)           get_6bit( &state->six_state, 2  );
    result->userid       = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->spare1       = (char)           get_6bit( &state->six_state, 2  );
    result->destid1      = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->msgid1_1     = (char)           get_6bit( &state->six_state, 6  );
    result->offset1_1    = (int)            get_6bit( &state->six_state, 12 );
    result->num_reqs     = 1;
    
    if( length > 88 )
    {
        result->spare2    = (char)  get_6bit( &state->six_state, 2 );
        result->msgid1_2  = (char)  get_6bit( &state->six_state, 6  );
        result->offset1_2 = (int)   get_6bit( &state->six_state, 12 );
        result->num_reqs  = 2;
    }
    if( length == 160 )
    {
        result->spare3    = (char)          get_6bit( &state->six_state, 2  );
        result->destid2   = (unsigned long) get_6bit( &state->six_state, 30 );
        result->msgid2_1  = (char)          get_6bit( &state->six_state, 6  );
        result->offset2_1 = (int)           get_6bit( &state->six_state, 12 );
        result->spare4    = (char)          get_6bit( &state->six_state, 2  );
        result->num_reqs  = 3;
    }

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 16 into an aismsg_16 structure

    Assigned mode command

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error

    Depending on the length of the message some of the fields may be 0.
    result->num_cmds has the number of commands received.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_16( ais_state *state, aismsg_16 *result )
{
    int length;

    if( !state )
        return 1;
    if( !result )
        return 1;

    /* Check the length of the packet */
    length = strlen( state->six_state.bits ) * 6;
    if( (length != 96) && (length != 144) )
        return 2;

    /* Clear it all to zeros */
    memset( result, 0, sizeof( aismsg_16 ));

    result->msgid = state->msgid;

    /* Parse the Message 16 */
    result->repeat       = (char)           get_6bit( &state->six_state, 2  );
    result->userid       = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->spare1       = (char)           get_6bit( &state->six_state, 2  );
    result->destid_a     = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->offset_a     = (int)            get_6bit( &state->six_state, 12 );
    result->increment_a  = (int)            get_6bit( &state->six_state, 10 );
    result->num_cmds     = 1;
    
    if( length == 144 )
    {
        result->destid_b     = (unsigned long)  get_6bit( &state->six_state, 30 );
        result->offset_b     = (int)            get_6bit( &state->six_state, 12 );
        result->increment_b  = (int)            get_6bit( &state->six_state, 10 );
        result->spare2       = (char)           get_6bit( &state->six_state, 4 );
        result->num_cmds     = 2;
    }

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 17 into an aismsg_17 structure

    GNSS Binary Broadcast Message

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return:
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error

    Note: the binary payload of the message is left in its 6-bit ASCII
          form when stored into result->data. This allows the data to
          be passed to the sixbit module for parsing.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_17( ais_state *state, aismsg_17 *result )
{
    int length;

    if( !state )
        return 1;
    if( !result )
        return 1;

    length = strlen(state->six_state.bits) * 6;
    if( (length < 80) || (length > 816) )
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_17 ));

    result->msgid = state->msgid;

    /* Parse the Message 17 */
    result->repeat       = (char)           get_6bit( &state->six_state, 2  );
    result->userid       = (unsigned long)  get_6bit( &state->six_state, 30 );
    result->spare1       = (char)           get_6bit( &state->six_state, 2  );
    result->longitude    = (long)           get_6bit( &state->six_state, 18 );
    result->latitude     = (long)           get_6bit( &state->six_state, 17 );
    result->spare2       = (char)           get_6bit( &state->six_state, 5  );
    result->msg_type     = (char)           get_6bit( &state->six_state, 6  );
    result->station_id   = (int)            get_6bit( &state->six_state, 10 );
    result->z_count      = (int)            get_6bit( &state->six_state, 13 );
    result->seq_num      = (char)           get_6bit( &state->six_state, 3  );
    result->num_words    = (char)           get_6bit( &state->six_state, 5  );
    result->health       = (char)           get_6bit( &state->six_state, 3  );

    /* Store the remaining payload of the packet for further processing */
	result->data = state->six_state;

    /* Convert the position to signed value */
    conv_pos( &result->latitude, &result->longitude);

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 18 into an aismsg_18 structure

    Class B Position Report

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return:
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error
      
     Note that the latitude and longitude are converted to signed values
     before being returned.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_18( ais_state *state, aismsg_18 *result )
{
    if( !state )
        return 1;
    if( !result )
        return 1;

    if (strlen(state->six_state.bits) != 28)
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_18 ));

    result->msgid = state->msgid;

    /* Parse the Message 18 */
    result->repeat         = (char)          get_6bit( &state->six_state, 2  );
    result->userid         = (unsigned long) get_6bit( &state->six_state, 30 );
    result->regional1      = (unsigned char) get_6bit( &state->six_state, 8  );
    result->sog            = (int)           get_6bit( &state->six_state, 10 );
    result->pos_acc        = (char)          get_6bit( &state->six_state, 1  );
    result->longitude      = (long)          get_6bit( &state->six_state, 28 );
    result->latitude       = (long)          get_6bit( &state->six_state, 27 );
    result->cog            = (int)           get_6bit( &state->six_state, 12 );
    result->true           = (int)           get_6bit( &state->six_state, 9  );
    result->utc_sec        = (char)          get_6bit( &state->six_state, 6  );
    result->regional2      = (char)          get_6bit( &state->six_state, 2  );
    result->unit_flag      = (char)          get_6bit( &state->six_state, 1  );
    result->display_flag   = (char)          get_6bit( &state->six_state, 1  );
    result->dsc_flag       = (char)          get_6bit( &state->six_state, 1  );
    result->band_flag      = (char)          get_6bit( &state->six_state, 1  );
    result->msg22_flag     = (char)          get_6bit( &state->six_state, 1  );
    result->mode_flag      = (char)          get_6bit( &state->six_state, 1  );
    result->raim           = (char)          get_6bit( &state->six_state, 1  );
    result->comm_state     = (char)          get_6bit( &state->six_state, 1  );

    if( result->comm_state == 0 )
    {
        result->sotdma.sync_state   = (char)  get_6bit( &state->six_state, 2  );
        result->sotdma.slot_timeout = (char)  get_6bit( &state->six_state, 3  );
        result->sotdma.sub_message  = (int)   get_6bit( &state->six_state, 14 );
    } else {
        result->itdma.sync_state    = (char)  get_6bit( &state->six_state, 2  );
        result->itdma.slot_inc      = (int)   get_6bit( &state->six_state, 13 );
        result->itdma.num_slots     = (char)  get_6bit( &state->six_state, 3  );
        result->itdma.keep_flag     = (char)  get_6bit( &state->six_state, 1  );
    }

    /* Convert the position to signed value */
    conv_pos( &result->latitude, &result->longitude);

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 19 into an aismsg_19 structure

    Extended Class B Position Report

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return:
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error

    Note that the latitude and longitude are converted to signed values
    before being returned. And the ship's name is converted to ASCII 
    before being stored.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_19( ais_state *state, aismsg_19 *result )
{
    int i;

    if( !state )
        return 1;
    if( !result )
        return 1;

    if (strlen(state->six_state.bits) != 52)
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_19 ));

    result->msgid = state->msgid;

    /* Parse the Message 19 */
    result->repeat         = (char)          get_6bit( &state->six_state, 2  );
    result->userid         = (unsigned long) get_6bit( &state->six_state, 30 );
    result->regional1      = (unsigned char) get_6bit( &state->six_state, 8  );
    result->sog            = (int)           get_6bit( &state->six_state, 10 );
    result->pos_acc        = (char)          get_6bit( &state->six_state, 1  );
    result->longitude      = (long)          get_6bit( &state->six_state, 28 );
    result->latitude       = (long)          get_6bit( &state->six_state, 27 );
    result->cog            = (int)           get_6bit( &state->six_state, 12 );
    result->true           = (int)           get_6bit( &state->six_state, 9  );
    result->utc_sec        = (char)          get_6bit( &state->six_state, 6  );
    result->regional2      = (char)          get_6bit( &state->six_state, 4  );

    /* Get the Ship Name, convert to ASCII */
    i = 0;
    while( i != 20 )
    {
        result->name[i] = ais2ascii( (char) get_6bit( &state->six_state, 6 ));
        i++;
    }
    result->name[i] = 0;
   
    result->ship_type    = (unsigned char)  get_6bit( &state->six_state, 8  );
    result->dim_bow      = (int)            get_6bit( &state->six_state, 9  );
    result->dim_stern    = (int)            get_6bit( &state->six_state, 9  );
    result->dim_port     = (char)           get_6bit( &state->six_state, 6  );
    result->dim_starboard= (char)           get_6bit( &state->six_state, 6  );
    result->pos_type     = (char)           get_6bit( &state->six_state, 4  );
    result->raim         = (char)           get_6bit( &state->six_state, 1  );
    result->dte          = (char)           get_6bit( &state->six_state, 1  );
    result->spare        = (char)           get_6bit( &state->six_state, 5  );

    /* Convert the position to signed value */
    conv_pos( &result->latitude, &result->longitude);

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 20 into an aismsg_20 structure

    Data link management message

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return:
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error

    Depending on the length of the message some fields may be 0.
    result->num_cmds will have the number of commands received.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_20( ais_state *state, aismsg_20 *result )
{
    int length;

    if( !state )
        return 1;
    if( !result )
        return 1;

    length = strlen(state->six_state.bits) * 6;
    if( (length < 72) || (length > 162) )
        return 2;

    /* Clear the result */
    memset( result, 0, sizeof( aismsg_20 ));

    result->msgid = state->msgid;

    /* Parse the Message 20 */
    result->repeat         = (char)          get_6bit( &state->six_state, 2  );
    result->userid         = (unsigned long) get_6bit( &state->six_state, 30 );
    result->spare1         = (unsigned char) get_6bit( &state->six_state, 2  );
    result->offset1        = (unsigned int)  get_6bit( &state->six_state, 12 );
    result->slots1         = (unsigned char) get_6bit( &state->six_state, 4  );
    result->timeout1       = (unsigned char) get_6bit( &state->six_state, 3  );
    result->increment1     = (unsigned int)  get_6bit( &state->six_state, 11 );
    result->num_cmds       = 1;

    if( length > 72 )
    {
        result->offset2    = (unsigned int)  get_6bit( &state->six_state, 12 );
        result->slots2     = (unsigned char) get_6bit( &state->six_state, 4  );
        result->timeout2   = (unsigned char) get_6bit( &state->six_state, 3  );
        result->increment2 = (unsigned int)  get_6bit( &state->six_state, 11 );
        result->num_cmds   = 2;
    }

    if( length > 104 )
    {
        result->offset3    = (unsigned int)  get_6bit( &state->six_state, 12 );
        result->slots3     = (unsigned char) get_6bit( &state->six_state, 4  );
        result->timeout3   = (unsigned char) get_6bit( &state->six_state, 3  );
        result->increment3 = (unsigned int)  get_6bit( &state->six_state, 11 );
        result->num_cmds   = 3;
    }

    if( length > 136 )
    {
        result->offset4    = (unsigned int)  get_6bit( &state->six_state, 12 );
        result->slots4     = (unsigned char) get_6bit( &state->six_state, 4  );
        result->timeout4   = (unsigned char) get_6bit( &state->six_state, 3  );
        result->increment4 = (unsigned int)  get_6bit( &state->six_state, 11 );
        result->num_cmds   = 4;
    }

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 21 into an aismsg_21 structure

    Aids To Navigation position report (AtoN)

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return:
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error

    Note that the latitude and longitude are converted to signed values
    before being returned. And the AtoN name and extended name are converted
    to ASCII before storage.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_21( ais_state *state, aismsg_21 *result )
{
    int length;
    int i;

    if( !state )
        return 1;
    if( !result )
        return 1;

    length = strlen(state->six_state.bits) * 6;
    if( (length < 272) || (length > 360) )
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_21 ));

    result->msgid = state->msgid;

    /* Parse the Message 21 */
    result->repeat         = (char)          get_6bit( &state->six_state, 2  );
    result->userid         = (unsigned long) get_6bit( &state->six_state, 30 );
    result->aton_type      = (char)          get_6bit( &state->six_state, 5  );

    /* Get the AtoN name, convert to ASCII */
    i = 0;
    while( i != 20 )
    {
        result->name[i] = ais2ascii( (char) get_6bit( &state->six_state, 6 ));
        i++;
    }
    result->name[i] = 0;

    result->pos_acc       = (char)          get_6bit( &state->six_state, 1  );
    result->longitude     = (long)          get_6bit( &state->six_state, 28 );
    result->latitude      = (long)          get_6bit( &state->six_state, 27 );
    result->dim_bow       = (int)           get_6bit( &state->six_state, 9  );
    result->dim_stern     = (int)           get_6bit( &state->six_state, 9  );
    result->dim_port      = (char)          get_6bit( &state->six_state, 6  );
    result->dim_starboard = (char)          get_6bit( &state->six_state, 6  );
    result->pos_type      = (char)          get_6bit( &state->six_state, 4  );
    result->utc_sec       = (char)          get_6bit( &state->six_state, 6  );
    result->off_position  = (char)          get_6bit( &state->six_state, 1  );
    result->regional      = (unsigned char) get_6bit( &state->six_state, 8  );
    result->raim          = (char)          get_6bit( &state->six_state, 1  );
    result->virtual       = (char)          get_6bit( &state->six_state, 1  );
    result->assigned      = (char)          get_6bit( &state->six_state, 1  );
    result->spare1        = (char)          get_6bit( &state->six_state, 1  );

    if( length > 272 )
    {
        /* Get the extended name 1-14 characters */
        i = 0;
        while( i < ((length-272) / 6) )
        {
            result->name_ext[i] = ais2ascii( (char) get_6bit( &state->six_state, 6 ));
            i++;
        }
        result->name_ext[i] = 0;
    }

    /* Convert the position to signed value */
    conv_pos( &result->latitude, &result->longitude);

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 22 into an aismsg_22 structure

    Channel Management

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error

    Check the addressed flag to decide whether to use the positions in
    NE and SW or the addresses in addressed_1 and addressed_2

    Note that the latitudes and longitudes are converted to signed values
    before being returned and that they are converted to 1/10000 minute
    format from 1/1000 minute as received.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_22( ais_state *state, aismsg_22 *result )
{
    if( !state )
        return 1;
    if( !result )
        return 1;

    if (strlen(state->six_state.bits) != 28)
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_22 ));

    result->msgid = state->msgid;

    /* Parse the Message 22 */
    result->repeat         = (char)          get_6bit( &state->six_state, 2  );
    result->userid         = (unsigned long) get_6bit( &state->six_state, 30 );
    result->spare1         = (char)          get_6bit( &state->six_state, 1  );
    result->channel_a      = (int)           get_6bit( &state->six_state, 12 );
    result->channel_b      = (int)           get_6bit( &state->six_state, 12 );
    result->txrx_mode      = (char)          get_6bit( &state->six_state, 4  );
    result->power          = (char)          get_6bit( &state->six_state, 1  );
    result->NE_longitude   = (long)          get_6bit( &state->six_state, 18 );
    result->NE_latitude    = (long)          get_6bit( &state->six_state, 17 );
    result->SW_longitude   = (long)          get_6bit( &state->six_state, 18 );
    result->SW_latitude    = (long)          get_6bit( &state->six_state, 17 );
    result->addressed      = (char)          get_6bit( &state->six_state, 1  );
    result->bw_a           = (char)          get_6bit( &state->six_state, 1  );
    result->bw_b           = (char)          get_6bit( &state->six_state, 1  );
    result->tz_size        = (char)          get_6bit( &state->six_state, 3  );

    /* Is the position actually an address? */
    if( result->addressed )
    {
        /* Convert the positions to addresses */
        result->addressed_1 = (result->NE_longitude << 12) + (result->NE_latitude >> 5);
        result->addressed_2 = (result->SW_longitude << 12) + (result->SW_latitude >> 5);
    } else {
        /* Convert 1/10 to 1/10000 before conversion */
        result->NE_longitude *= 1000;
        result->NE_latitude  *= 1000;

        result->SW_longitude *= 1000;
        result->SW_latitude  *= 1000;
    }

    /* Convert the position to signed value */
    conv_pos( &result->NE_latitude, &result->NE_longitude);
    conv_pos( &result->SW_latitude, &result->SW_longitude);

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 23 into an aismsg_23 structure

    Group Assignment Command

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled
   
    return:
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error

    Note that the latitudes and longitudes are converted to signed values
    before being returned and that they are converted to 1/10000 minute
    format from 1/1000 minute as received.
      
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_23( ais_state *state, aismsg_23 *result )
{
    if( !state )
        return 1;
    if( !result )
        return 1;

    if (strlen(state->six_state.bits) != 28)
        return 2;

    /* Clear out the structure first */
    memset( result, 0, sizeof( aismsg_23 ));

    result->msgid = state->msgid;

    /* Parse the Message 23 */
    result->repeat         = (char)          get_6bit( &state->six_state, 2  );
    result->userid         = (unsigned long) get_6bit( &state->six_state, 30 );
    result->spare1         = (char)          get_6bit( &state->six_state, 2  );
    result->NE_longitude   = (long)          get_6bit( &state->six_state, 18 );
    result->NE_latitude    = (long)          get_6bit( &state->six_state, 17 );
    result->SW_longitude   = (long)          get_6bit( &state->six_state, 18 );
    result->SW_latitude    = (long)          get_6bit( &state->six_state, 17 );
    result->station_type   = (char)          get_6bit( &state->six_state, 4  );
    result->ship_type      = (unsigned char) get_6bit( &state->six_state, 8  );
    result->spare2         = (unsigned long) get_6bit( &state->six_state, 22 );
    result->txrx_mode      = (char)          get_6bit( &state->six_state, 2  );
    result->report_interval= (char)          get_6bit( &state->six_state, 4  );
    result->quiet_time     = (char)          get_6bit( &state->six_state, 4  );
    result->spare3         = (char)          get_6bit( &state->six_state, 6  );

    result->NE_longitude *= 1000;
    result->NE_latitude  *= 1000;
    result->SW_longitude *= 1000;
    result->SW_latitude  *= 1000;

    /* Convert the position to signed value */
    conv_pos( &result->NE_latitude, &result->NE_longitude);
    conv_pos( &result->SW_latitude, &result->SW_longitude);

    return 0;
}


/* ----------------------------------------------------------------------- */
/** Parse an AIS message 24 into an aismsg_24 structure

    Class B"CS" Static Data Report

    \param state    pointer to ais_state
    \param result   pointer to parsed message result structure to be filled.

    return:
      - 0 if no errors
      - 1 if there is an error
      - 2 if there is a packet length error
      - 3 if unknown part number

    NOTE! The result structure needs to be cleared before use. Because 
    message 24 is a 2 part message the parse function doesn't clear the 
    structure before filling it. You should do this:

    \code
       memset( result, 0, sizeof( aismsg_24 ));
    \endcode

	before passing a message 24A to the parse_ais_24() function.

    Message 24 is a 2 part message. The first part only contains the MMSI
    and the ship name. The second message contains the ship dimensions, 
    etc.

    Check the result->part_number field to determine which message this 
    is. The same structure is used for both messages and the result->flags
    field will have a 0x03 in it when both messages have been parsed.

    The ship name, vendor id and callsign are all converted to ASCII before
    storage.
*/
/* ----------------------------------------------------------------------- */
int __stdcall  parse_ais_24( ais_state *state, aismsg_24 *result )
{
    int length;
    int i;

    if( !state )
        return 1;
    if( !result )
        return 1;

    length = strlen(state->six_state.bits) * 6;
    if( (length != 162) && (length != 168) )
        return 2;

    result->msgid = state->msgid;

    /* Parse the Message 24 */
    result->repeat         = (char)          get_6bit( &state->six_state, 2  );
    result->userid         = (unsigned long) get_6bit( &state->six_state, 30 );
    result->part_number    = (char)          get_6bit( &state->six_state, 2  );

    if( result->part_number == 0 )
    {
        /* Parse 24A */
        /* Get the Ship Name, convert to ASCII */
        i = 0;
        while( i != 20 )
        {
            result->name[i] = ais2ascii( (char) get_6bit( &state->six_state, 6 ));
            i++;
        }
        result->name[i] = 0;

        /* Indicate reception of part A */
        result->flags |= 0x01;
    } else if( result->part_number == 1 ) {
        /* Parse 24B */
        result->ship_type = (unsigned char) get_6bit( &state->six_state, 8  );

        i = 0;
        while( i != 7 )
        {
            result->vendor_id[i] = ais2ascii( (char) get_6bit( &state->six_state, 6 ));
            i++;
        }
        result->vendor_id[i] = 0;

        i = 0;
        while( i != 7 )
        {
            result->callsign[i] = ais2ascii( (char) get_6bit( &state->six_state, 6 ));
            i++;
        }
        result->callsign[i] = 0;

        result->dim_bow      = (int)   get_6bit( &state->six_state, 9  );
        result->dim_stern    = (int)   get_6bit( &state->six_state, 9  );
        result->dim_port     = (char)  get_6bit( &state->six_state, 6  );
        result->dim_starboard= (char)  get_6bit( &state->six_state, 6  );
        result->spare        = (char)  get_6bit( &state->six_state, 6  );

        /* Indicate reception of part A */
        result->flags |= 0x02;
    } else {
        return 3;
    }
    return 0;
}



#define AIS_MSG_PART2_FLAG 0x100

static unsigned char convtab[] = { "0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVW`abcdefghijklmnopqrstuvw" };
static unsigned char contab1[] = { 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2a,0x2b,0x2c,0x2d,0x2e,0x2f,
0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,
0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,
0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff };

static void ais_addbits(unsigned char *message,
	unsigned int   start,
	unsigned int   len,
	uint64_t       data)
{
	unsigned int  l;
	unsigned int  pos;
	uint64_t      mask;
	unsigned int  mask1;

	mask = 0x1;
	pos = (start + len - 1) / 6;
	mask1 = 0x20;
	mask1 = mask1 >> ((start + len - 1) % 6);

	if (len == 0) {
		return;
	}

	for (l = 0; l<len; l++) {
		if (data & mask) {
			message[pos] |= mask1;
		}
		mask <<= 1;
		mask1 <<= 1;
		if (mask1 == 0x40) {
			pos -= 1;
			mask1 = 0x1;
		}
	}
	return;
}

/*
static void ais_addchar(unsigned char *message,
	unsigned int   start,
	unsigned int   len,
	char          *data)
{
	unsigned int l;
	unsigned int flag;

	for (l = 0, flag = 0; l<len; l++) {
		unsigned char a, b;

		a = (unsigned char)data[l];
		if (a == (unsigned char)'\0') {
			flag = 1;
		}
		if (flag == 0) {
			b = contab1[a & 0x7f];
		}
		else {
			b = (unsigned char) '\0';
		}
		ais_addbits(message, start + 6 * l, 6, (uint64_t)b);
	}
	return;
}*/


/*
static void ais_adddata(unsigned char *message,
	unsigned int   start,
	unsigned int   len,
	char          *data)
{
	unsigned int l;

	for (l = 0; l<len; l++) {
		ais_addbits(message, start + 6 * l, 6, (uint64_t)data[l]);
	}
	return;
}*/


static void ais_binary_to_ascii(unsigned char *message, unsigned int len)
{
	unsigned int l;

	if (len == 0) {
		message[0] = '\0';
		return;
	}

	for (l = 0; l<len; l += 6) {
		message[l / 6] = convtab[message[l / 6] & 0x3f];
	}
	return;
}

int ais_binary_encode_1( aismsg_1 *ais,
	char *msg,
	int flag)
{

	if (flag != 0) 
		flag = AIS_MSG_PART2_FLAG;
	unsigned char* message = (unsigned char*) msg;
	int pos = 0;
	ais_addbits(message, pos, 6, (uint64_t)ais->msgid); pos += 6;
	ais_addbits(message, pos, 2, (uint64_t)ais->repeat); pos += 2;
	ais_addbits(message, pos, 30, (uint64_t)ais->userid); pos += 30;
	ais_addbits(message, pos, 4, (uint64_t)ais->nav_status); pos += 4;
	ais_addbits(message, pos, 8, (uint64_t)ais->rot); pos += 8;
	ais_addbits(message, pos, 10, (uint64_t)ais->sog); pos += 10;
	ais_addbits(message, pos, 1, (uint64_t)ais->pos_acc); pos += 1;
	ais_addbits(message, pos, 28, (uint64_t)ais->longitude); pos += 28;
	ais_addbits(message, pos, 27, (uint64_t)ais->latitude); pos += 27;
	ais_addbits(message, pos, 12, (uint64_t)ais->cog); pos += 12;
	ais_addbits(message, pos, 9, (uint64_t)ais->true); pos += 9;
	ais_addbits(message, pos, 6, (uint64_t)ais->utc_sec); pos += 6;
	ais_addbits(message, pos, 4, (uint64_t)ais->regional); pos += 4;
	ais_addbits(message, pos, 1, (uint64_t)ais->spare); pos += 1;
	ais_addbits(message, pos, 1, (uint64_t)ais->raim); pos += 1;
	ais_addbits(message, pos, 2, (uint64_t)ais->sync_state); pos += 2;
	ais_addbits(message, pos, 3, (uint64_t)ais->slot_timeout); pos += 3;
	ais_addbits(message, pos, 14, (uint64_t)ais->sub_message); pos += 14; 
	ais_binary_to_ascii(message, pos);
    msg = (char*) message;
	return 1;
}
/*
int ais_binary_encode_2(aismsg_2 *ais,
	char *message,
	int flag)
{
	if (flag != 0) 
		flag = AIS_MSG_PART2_FLAG;

	int pos = 0;
	ais_addbits(message, pos, 6, (uint64_t)ais->msgid); pos += 6;
	ais_addbits(message, pos, 2, (uint64_t)ais->repeat); pos += 2;
	ais_addbits(message, pos, 30, (uint64_t)ais->userid); pos += 30;
	ais_addbits(message, pos, 4, (uint64_t)ais->nav_status); pos += 4;
	ais_addbits(message, pos, 8, (uint64_t)ais->rot); pos += 8;
	ais_addbits(message, pos, 10, (uint64_t)ais->sog); pos += 10;
	ais_addbits(message, pos, 1, (uint64_t)ais->pos_acc); pos += 1;
	ais_addbits(message, pos, 28, (uint64_t)ais->longitude); pos += 28;
	ais_addbits(message, pos, 27, (uint64_t)ais->latitude); pos += 27;
	ais_addbits(message, pos, 12, (uint64_t)ais->cog); pos += 12;
	ais_addbits(message, pos, 9, (uint64_t)ais->true); pos += 9;
	ais_addbits(message, pos, 6, (uint64_t)ais->utc_sec); pos += 6;
	ais_addbits(message, pos, 4, (uint64_t)ais->regional); pos += 4;
	ais_addbits(message, pos, 1, (uint64_t)ais->spare); pos += 1;
	ais_addbits(message, pos, 1, (uint64_t)ais->raim); pos += 1;
	ais_addbits(message, pos, 2, (uint64_t)ais->sync_state); pos += 2;
	ais_addbits(message, pos, 3, (uint64_t)ais->slot_timeout); pos += 3;
	ais_addbits(message, pos, 14, (uint64_t)ais->sub_message); pos += 14;
	ais_binary_to_ascii(message, pos);
	return 1;
}

int ais_binary_encode_3(aismsg_3 *ais,
	char *message,
	int flag)
{
	if (flag != 0)
		flag = AIS_MSG_PART2_FLAG;
	int pos = 0;
	ais_addbits(message, pos, 6, (uint64_t)ais->msgid); pos += 6;
	ais_addbits(message, pos, 2, (uint64_t)ais->repeat); pos += 2;
	ais_addbits(message, pos, 30, (uint64_t)ais->userid); pos += 30;
	ais_addbits(message, pos, 4, (uint64_t)ais->nav_status); pos += 4;
	ais_addbits(message, pos, 8, (uint64_t)ais->rot); pos += 8;
	ais_addbits(message, pos, 10, (uint64_t)ais->sog); pos += 10;
	ais_addbits(message, pos, 1, (uint64_t)ais->pos_acc); pos += 1;
	ais_addbits(message, pos, 28, (uint64_t)ais->longitude); pos += 28;
	ais_addbits(message, pos, 27, (uint64_t)ais->latitude); pos += 27;
	ais_addbits(message, pos, 12, (uint64_t)ais->cog); pos += 12;
	ais_addbits(message, pos, 9, (uint64_t)ais->true); pos += 9;
	ais_addbits(message, pos, 6, (uint64_t)ais->utc_sec); pos += 6;
	ais_addbits(message, pos, 4, (uint64_t)ais->regional); pos += 4;
	ais_addbits(message, pos, 1, (uint64_t)ais->spare); pos += 1;
	ais_addbits(message, pos, 1, (uint64_t)ais->raim); pos += 1;
	ais_addbits(message, pos, 2, (uint64_t)ais->sync_state); pos += 2;
	ais_addbits(message, pos, 13, (uint64_t)ais->slot_increment); pos += 13;
	ais_addbits(message, pos, 3, (uint64_t)ais->num_slots); pos += 3;
	ais_addbits(message, pos, 1, (uint64_t)ais->keep); pos += 1;
	ais_binary_to_ascii(message, pos);
	return 1;
}

int ais_binary_encode_4(aismsg_4 *ais,
	char *message,
	int flag)
{
	if (flag != 0)
		flag = AIS_MSG_PART2_FLAG;

	int pos = 0;
	ais_addbits(message, pos, 6, (uint64_t)ais->msgid); pos += 6;
	ais_addbits(message, pos, 2, (uint64_t)ais->repeat); pos += 2;
	ais_addbits(message, pos, 30, (uint64_t)ais->userid); pos += 30;
	ais_addbits(message, pos, 14, (uint64_t)ais->utc_year); pos += 14;
	ais_addbits(message, pos, 4, (uint64_t)ais->utc_month); pos += 4;
	ais_addbits(message, pos, 5, (uint64_t)ais->utc_day); pos += 5;
	ais_addbits(message, pos, 5, (uint64_t)ais->utc_hour); pos += 5;
	ais_addbits(message, pos, 6, (uint64_t)ais->utc_minute); pos += 6;
	ais_addbits(message, pos, 6, (uint64_t)ais->utc_second); pos += 6;
	ais_addbits(message, pos, 1, (uint64_t)ais->pos_acc); pos += 1;
	ais_addbits(message, pos, 28, (uint64_t)ais->longitude); pos += 28;
	ais_addbits(message, pos, 27, (uint64_t)ais->latitude); pos += 27;
	ais_addbits(message, pos, 4, (uint64_t)ais->pos_type); pos += 4;
	ais_addbits(message, pos, 10, (uint64_t)ais->spare); pos += 10;
	ais_addbits(message, pos, 1, (uint64_t)ais->raim); pos += 1;
	ais_addbits(message, pos, 2, (uint64_t)ais->sync_state); pos += 2;
	ais_addbits(message, pos, 3, (uint64_t)ais->slot_timeout); pos += 3;
	ais_addbits(message, pos, 14, (uint64_t)ais->sub_message); pos += 14;
	ais_binary_to_ascii(message, pos);
	return 1;
}

int ais_binary_encode_5(aismsg_5 *ais,
	char *message,
	int flag)
{

	if (flag != 0)
		flag = AIS_MSG_PART2_FLAG;

	int pos = 0;
	ais_addbits(message, pos, 6, (uint64_t)ais->msgid); pos += 6;
	ais_addbits(message, pos, 2, (uint64_t)ais->repeat); pos += 2;
	ais_addbits(message, pos, 30, (uint64_t)ais->userid); pos += 30;
	ais_addbits(message, pos, 4, (uint64_t)ais->nav_status); pos += 4;
	ais_addbits(message, pos, 8, (uint64_t)ais->rot); pos += 8;
	ais_addbits(message, pos, 10, (uint64_t)ais->sog); pos += 10;
	ais_addbits(message, pos, 1, (uint64_t)ais->pos_acc); pos += 1;
	ais_addbits(message, pos, 28, (uint64_t)ais->longitude); pos += 28;
	ais_addbits(message, pos, 27, (uint64_t)ais->latitude); pos += 27;
	ais_addbits(message, pos, 12, (uint64_t)ais->cog); pos += 12;
	ais_addbits(message, pos, 9, (uint64_t)ais->true); pos += 9;
	ais_addbits(message, pos, 6, (uint64_t)ais->utc_sec); pos += 6;
	ais_addbits(message, pos, 4, (uint64_t)ais->regional); pos += 4;
	ais_addbits(message, pos, 1, (uint64_t)ais->spare); pos += 1;
	ais_addbits(message, pos, 1, (uint64_t)ais->raim); pos += 1;
	ais_addbits(message, pos, 2, (uint64_t)ais->sync_state); pos += 2;
	ais_addbits(message, pos, 3, (uint64_t)ais->slot_timeout); pos += 3;
	ais_addbits(message, pos, 14, (uint64_t)ais->sub_message); pos += 14;
	ais_binary_to_ascii(message, pos);
	return 1;
}*/

char* assemble_message(char* message)
{	
	unsigned char checksum ;
	char* ais_head = "!AIVDM,";
	if (strlen(message) <= (256 - 19))
	{
		char* ais_aux = "1,1,,B,";
		int len = 4 + strlen(ais_head) + strlen(ais_aux) + strlen(message);
		char* result = (char*)malloc(len);
		strcpy(result, ais_head);
		strcat(result, ais_aux);
		strcat(result, message);
		strcat(result, ",0*");

		nmea_checksum(result, &checksum);
		char* tail = bintohex(checksum);
		result[len - 1] = tail[0]; result[len] = tail[1];
		return result;
	}else printf("Tamaño inadecuado\n");
	return NULL;
}

char* bintohex(unsigned char checksum)
{	
	char tail[2];
	char abcHex[] = "0123456789ABCDEF";
	tail[0] = abcHex[checksum >> 4];
	checksum &= 0xF;
   	tail[1] = abcHex[checksum];
	return tail;
}
/*
unsigned int ais_binary_encode(struct ais_t *ais,
	unsigned char *bits,
	int flag)
{
	unsigned int len;

	len = 0;

	if (flag != 0) {
		flag = AIS_MSG_PART2_FLAG;
	}
	ais_addbits(bits, 0, 6, (uint64_t)ais->type);
	ais_addbits(bits, 6, 2, (uint64_t)ais->repeat);
	ais_addbits(bits, 8, 30, (uint64_t)ais->mmsi);
	switch (flag | ais->type) {
	case 1:	// Position Report 
	case 2:
	case 3:
		ais_addbits(bits, 38, 4, (uint64_t)ais->type1.status);
		ais_addbits(bits, 42, 8, (uint64_t)ais->type1.turn);
		ais_addbits(bits, 50, 10, (uint64_t)ais->type1.speed);
		ais_addbits(bits, 60, 1, (uint64_t)ais->type1.accuracy);
		ais_addbits(bits, 61, 28, (uint64_t)ais->type1.lon);
		ais_addbits(bits, 89, 27, (uint64_t)ais->type1.lat);
		ais_addbits(bits, 116, 12, (uint64_t)ais->type1.course);
		ais_addbits(bits, 128, 9, (uint64_t)ais->type1.heading);
		ais_addbits(bits, 137, 6, (uint64_t)ais->type1.second);
		ais_addbits(bits, 143, 2, (uint64_t)ais->type1.maneuver);
		//	ais_addbits(bits, 145,  3, (uint64_t)ais->type1.spare); 
		ais_addbits(bits, 148, 1, (uint64_t)ais->type1.raim);
		ais_addbits(bits, 149, 19, (uint64_t)ais->type1.radio);
		len = 149 + 19;
		break;
	case 4: 	// Base Station Report 
	case 11:	// UTC/Date Response
		ais_addbits(bits, 38, 14, (uint64_t)ais->type4.year);
		ais_addbits(bits, 52, 4, (uint64_t)ais->type4.month);
		ais_addbits(bits, 56, 5, (uint64_t)ais->type4.day);
		ais_addbits(bits, 61, 5, (uint64_t)ais->type4.hour);
		ais_addbits(bits, 66, 6, (uint64_t)ais->type4.minute);
		ais_addbits(bits, 72, 6, (uint64_t)ais->type4.second);
		ais_addbits(bits, 78, 1, (uint64_t)ais->type4.accuracy);
		ais_addbits(bits, 79, 28, (uint64_t)ais->type4.lon);
		ais_addbits(bits, 107, 27, (uint64_t)ais->type4.lat);
		ais_addbits(bits, 134, 4, (uint64_t)ais->type4.epfd);
		//	ais_addbits(bits, 138, 10, (uint64_t)ais->type4.spare); 
		ais_addbits(bits, 148, 1, (uint64_t)ais->type4.raim);
		ais_addbits(bits, 149, 19, (uint64_t)ais->type4.radio);
		len = 149 + 19;
		break;
	case 5:     // Ship static and voyage related data 
		ais_addbits(bits, 38, 2, (uint64_t)ais->type5.ais_version);
		ais_addbits(bits, 40, 30, (uint64_t)ais->type5.imo);
		ais_addchar(bits, 70, 7, ais->type5.callsign);
		ais_addchar(bits, 112, 20, ais->type5.shipname);
		ais_addbits(bits, 232, 8, (uint64_t)ais->type5.shiptype);
		ais_addbits(bits, 240, 9, (uint64_t)ais->type5.to_bow);
		ais_addbits(bits, 249, 9, (uint64_t)ais->type5.to_stern);
		ais_addbits(bits, 258, 6, (uint64_t)ais->type5.to_port);
		ais_addbits(bits, 264, 6, (uint64_t)ais->type5.to_starboard);
		ais_addbits(bits, 270, 4, (uint64_t)ais->type5.epfd);
		ais_addbits(bits, 274, 4, (uint64_t)ais->type5.month);
		ais_addbits(bits, 278, 5, (uint64_t)ais->type5.day);
		ais_addbits(bits, 283, 5, (uint64_t)ais->type5.hour);
		ais_addbits(bits, 288, 6, (uint64_t)ais->type5.minute);
		ais_addbits(bits, 294, 8, (uint64_t)ais->type5.draught);
		ais_addchar(bits, 302, 20, ais->type5.destination);
		ais_addbits(bits, 422, 1, (uint64_t)ais->type5.dte);
		//      ais_addbits(bits, 423,  1, (uint64_t)ais->type5.spare); 
		len = 423 + 1;
		break;
	case 9:     // Standard SAR Aircraft Position Report 
		ais_addbits(bits, 38, 12, (uint64_t)ais->type9.alt);
		ais_addbits(bits, 50, 10, (uint64_t)ais->type9.speed);
		ais_addbits(bits, 60, 1, (uint64_t)ais->type9.accuracy);
		ais_addbits(bits, 61, 28, (uint64_t)ais->type9.lon);
		ais_addbits(bits, 89, 27, (uint64_t)ais->type9.lat);
		ais_addbits(bits, 116, 12, (uint64_t)ais->type9.course);
		ais_addbits(bits, 128, 6, (uint64_t)ais->type9.second);
		ais_addbits(bits, 134, 8, (uint64_t)ais->type9.regional);
		ais_addbits(bits, 142, 1, (uint64_t)ais->type9.dte);
		//	ais_addbits(bits, 143,  3, (uint64_t)ais->type9.spare); 
		ais_addbits(bits, 146, 1, (uint64_t)ais->type9.assigned);
		ais_addbits(bits, 147, 1, (uint64_t)ais->type9.raim);
		ais_addbits(bits, 148, 19, (uint64_t)ais->type9.radio);
		len = 148 + 19;
		break;
	case 18:	// Standard Class B CS Position Report 
		ais_addbits(bits, 38, 8, (uint64_t)ais->type18.reserved);
		ais_addbits(bits, 46, 10, (uint64_t)ais->type18.speed);
		ais_addbits(bits, 56, 1, (uint64_t)ais->type18.accuracy);
		ais_addbits(bits, 57, 28, (uint64_t)ais->type18.lon);
		ais_addbits(bits, 85, 27, (uint64_t)ais->type18.lat);
		ais_addbits(bits, 112, 12, (uint64_t)ais->type18.course);
		ais_addbits(bits, 124, 9, (uint64_t)ais->type18.heading);
		ais_addbits(bits, 133, 6, (uint64_t)ais->type18.second);
		ais_addbits(bits, 139, 2, (uint64_t)ais->type18.regional);
		ais_addbits(bits, 141, 1, (uint64_t)ais->type18.cs);
		ais_addbits(bits, 142, 1, (uint64_t)ais->type18.display);
		ais_addbits(bits, 143, 1, (uint64_t)ais->type18.dsc);
		ais_addbits(bits, 144, 1, (uint64_t)ais->type18.band);
		ais_addbits(bits, 145, 1, (uint64_t)ais->type18.msg22);
		ais_addbits(bits, 146, 1, (uint64_t)ais->type18.assigned);
		ais_addbits(bits, 147, 1, (uint64_t)ais->type18.raim);
		ais_addbits(bits, 148, 20, (uint64_t)ais->type18.radio);
		len = 148 + 20;
		break;
	case 19:	// Extended Class B CS Position Report 
		ais_addbits(bits, 38, 8, (uint64_t)ais->type19.reserved);
		ais_addbits(bits, 46, 10, (uint64_t)ais->type19.speed);
		ais_addbits(bits, 56, 1, (uint64_t)ais->type19.accuracy);
		ais_addbits(bits, 57, 28, (uint64_t)ais->type19.lon);
		ais_addbits(bits, 85, 27, (uint64_t)ais->type19.lat);
		ais_addbits(bits, 112, 12, (uint64_t)ais->type19.course);
		ais_addbits(bits, 124, 9, (uint64_t)ais->type19.heading);
		ais_addbits(bits, 133, 6, (uint64_t)ais->type19.second);
		ais_addbits(bits, 139, 4, (uint64_t)ais->type19.regional);
		ais_addchar(bits, 143, 20, ais->type19.shipname);
		ais_addbits(bits, 263, 8, (uint64_t)ais->type19.shiptype);
		ais_addbits(bits, 271, 9, (uint64_t)ais->type19.to_bow);
		ais_addbits(bits, 280, 9, (uint64_t)ais->type19.to_stern);
		ais_addbits(bits, 289, 6, (uint64_t)ais->type19.to_port);
		ais_addbits(bits, 295, 6, (uint64_t)ais->type19.to_starboard);
		ais_addbits(bits, 299, 4, (uint64_t)ais->type19.epfd);
		ais_addbits(bits, 302, 1, (uint64_t)ais->type19.raim);
		ais_addbits(bits, 305, 1, (uint64_t)ais->type19.dte);
		ais_addbits(bits, 306, 1, (uint64_t)ais->type19.assigned);
		//      ais_addbits(bits, 307,  5, (uint64_t)ais->type19.spare); 
		len = 307 + 5;
		break;
	case 21:	// Aid-to-Navigation Report
		ais_addbits(bits, 38, 5, (uint64_t)ais->type21.aid_type);
		ais_addchar(bits, 43, 20, ais->type21.name);
		ais_addbits(bits, 163, 1, (uint64_t)ais->type21.accuracy);
		ais_addbits(bits, 164, 28, (uint64_t)ais->type21.lon);
		ais_addbits(bits, 192, 27, (uint64_t)ais->type21.lat);
		ais_addbits(bits, 219, 9, (uint64_t)ais->type21.to_bow);
		ais_addbits(bits, 228, 9, (uint64_t)ais->type21.to_stern);
		ais_addbits(bits, 237, 6, (uint64_t)ais->type21.to_port);
		ais_addbits(bits, 243, 6, (uint64_t)ais->type21.to_starboard);
		ais_addbits(bits, 249, 4, (uint64_t)ais->type21.epfd);
		ais_addbits(bits, 253, 6, (uint64_t)ais->type21.second);
		ais_addbits(bits, 259, 1, (uint64_t)ais->type21.off_position);
		ais_addbits(bits, 260, 8, (uint64_t)ais->type21.regional);
		ais_addbits(bits, 268, 1, (uint64_t)ais->type21.raim);
		ais_addbits(bits, 269, 1, (uint64_t)ais->type21.virtual_aid);
		ais_addbits(bits, 270, 1, (uint64_t)ais->type21.assigned);
		//     ais_addbits(bits, 271,  1, (uint64_t)ais->type21.spare);            
		len = 271 + 1;
		if (strlen(ais->type21.name) > 20) {
			unsigned int extralen = (unsigned int)(strlen(ais->type21.name) - 20);
			ais_addchar(bits, 272, extralen, ais->type21.name + 20);
			len += extralen * 6;
		}
		break;
	case 24:	// Class B CS Static Data Report Part 1 
		if (ais->type24.part == part_a) {
			ais_addbits(bits, 38, 2, (uint64_t)0);
			ais_addchar(bits, 40, 20, ais->type24.shipname);
			//         ais_addbits(bits, 160,  8, (uint64_t)ais->type24.a.spare); 
			len = 160;
		}
		break;
	case 24 | AIS_MSG_PART2_FLAG: // Class B CS Static Data Report Part 2 
		if ((ais->type24.part == part_b) || (ais->type24.part == both)) {
			ais_addbits(bits, 38, 2, (uint64_t)1);
			ais_addbits(bits, 40, 8, (uint64_t)ais->type24.shiptype);
			ais_addchar(bits, 48, 3, &ais->type24.vendorid[0]);
			ais_adddata(bits, 66, 3, &ais->type24.vendorid[3]);
			ais_addchar(bits, 90, 7, ais->type24.callsign);
			if (AIS_AUXILIARY_MMSI(ais->mmsi)) {
				ais_addbits(bits, 132, 30, (uint64_t)ais->type24.mothership_mmsi);
			}
			else {
				ais_addbits(bits, 132, 9, (uint64_t)ais->type24.dim.to_bow);
				ais_addbits(bits, 141, 9, (uint64_t)ais->type24.dim.to_stern);
				ais_addbits(bits, 150, 6, (uint64_t)ais->type24.dim.to_port);
				ais_addbits(bits, 156, 6, (uint64_t)ais->type24.dim.to_starboard);
			}
			//          ais_addbits(bits, 162,  6,          ais->type24.b.spare); 
			len = 162 + 6;
		}
		break;
	case 27:	// Long Range AIS Broadcast message 
		ais_addbits(bits, 38, 1, (uint64_t)ais->type27.accuracy);
		ais_addbits(bits, 39, 1, (uint64_t)ais->type27.raim);
		ais_addbits(bits, 40, 4, (uint64_t)ais->type27.status);
		ais_addbits(bits, 44, 18, (uint64_t)ais->type27.lon);
		ais_addbits(bits, 62, 17, (uint64_t)ais->type27.lat);
		ais_addbits(bits, 79, 6, (uint64_t)ais->type27.speed);
		ais_addbits(bits, 85, 9, (uint64_t)ais->type27.course);
		ais_addbits(bits, 94, 1, (uint64_t)ais->type27.gnss);
		break;
	}
	ais_binary_to_ascii(bits, len);
	return len;
}
#endif // AIVDM_ENABLE 
*/
