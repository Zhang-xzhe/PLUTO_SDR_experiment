/*
 * libiio - AD9361 IIO streaming example
 *
 * Copyright (C) 2014 IABG mbH
 * Author: Michael Feilen <feilen_at_iabg.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 **/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <iio.h>
#include<unistd.h>

/* helper macros */
#define KHZ(x) ((long long)(x*1000.0+0.5))
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))

#define IIO_ENSURE(expr) { \
	if (!(expr)) { \
		(void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
		(void) abort(); \
	} \
}

/* RX is input, TX is output */
enum iodev { RX, TX };

/* common RX and TX streaming params */
struct stream_cfg {
	long long bw_hz; // Analog banwidth in Hz
	long long fs_hz; // Baseband sample rate in Hz
	long long lo_hz; // Local oscillator frequency in Hz
	double gain;     // gain in dB
	const char* gain_mode; // gain style
	const char* rfport;    // Port name
};

/* static scratch mem for strings */
static char tmpstr[64];

/* IIO structs required for streaming */
static struct iio_context *ctx   = NULL;
static struct iio_channel *rx0_i = NULL;
static struct iio_channel *rx0_q = NULL;
static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
static struct iio_buffer  *rxbuf = NULL;
static struct iio_buffer  *txbuf = NULL;

static bool stop;

/* cleanup and exit */
static void shutdown()
{
	printf("* Destroying buffers\n");
	if (rxbuf) { iio_buffer_destroy(rxbuf); }
	if (txbuf) { iio_buffer_destroy(txbuf); }

	printf("* Disabling streaming channels\n");
	if (rx0_i) { iio_channel_disable(rx0_i); }
	if (rx0_q) { iio_channel_disable(rx0_q); }
	if (tx0_i) { iio_channel_disable(tx0_i); }
	if (tx0_q) { iio_channel_disable(tx0_q); }

	printf("* Destroying context\n");
	if (ctx) { iio_context_destroy(ctx); }
	exit(0);
}

static void handle_sig(int sig)
{
	printf("Waiting for process to finish... Got signal %d\n", sig);
	stop = true;
}

/* check return value of attr_write function */
static void errchk(int v, const char* what) {
	 if (v < 0) { fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what); shutdown(); }
}

/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
	errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* write attribute: double int */
static void wr_ch_double(struct iio_channel *chn, const char* what, long long val)
{
	errchk(iio_channel_attr_write_double(chn, what, val), what);
}

/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what, const char* str)
{
	errchk(iio_channel_attr_write(chn, what, str), what);
}

/* helper function generating channel names */
static char* get_ch_name(const char* type, int id)         //???????????????
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	return tmpstr;
}

/* returns ad9361 phy device */
static struct iio_device* get_ad9361_phy(void)
{
	struct iio_device *dev =  iio_context_find_device(ctx, "ad9361-phy");
	IIO_ENSURE(dev && "No ad9361-phy found");
	return dev;
}

/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(enum iodev d, struct iio_device **dev)
{
	switch (d) {
	case TX: *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc"); return *dev != NULL;
	case RX: *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");  return *dev != NULL;
	default: IIO_ENSURE(0); return false;
	}
}

/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn)
{
	*chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
	if (!*chn)
		*chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
	return *chn != NULL;
}

/* finds AD9361 phy IIO configuration channel with id chid */
static bool get_phy_chan(enum iodev d, int chid, struct iio_channel **chn)
{
	switch (d) {
	case RX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("voltage", chid), false); return *chn != NULL;  //??????????????????
	case TX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("voltage", chid), true);  return *chn != NULL;
	default: IIO_ENSURE(0); return false;
	}
}

/* finds AD9361 local oscillator IIO configuration channels */
static bool get_lo_chan(enum iodev d, struct iio_channel **chn)
{
	switch (d) {
	 // LO chan is always output, i.e. true
	case RX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("altvoltage", 0), true); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("altvoltage", 1), true); return *chn != NULL;
	default: IIO_ENSURE(0); return false;
	}
}

/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct stream_cfg *cfg, enum iodev type, int chid)
{
	struct iio_channel *chn = NULL;   //?????????????????????

	// Configure phy and lo channels
	printf("* Acquiring AD9361 phy channel %d\n", chid);
	if (!get_phy_chan(type, chid, &chn)) {	return false; }    //?????????????????????
	wr_ch_str(chn, "rf_port_select",     cfg->rfport);
	wr_ch_str(chn, "gain_control_mode",     cfg->gain_mode);
	wr_ch_lli(chn, "rf_bandwidth",       cfg->bw_hz);
	wr_ch_double(chn, "hardwaregain",       cfg->gain);
	wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);


	// Configure LO channel
	printf("* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
	if (!get_lo_chan(type, &chn)) { return false; }
	wr_ch_lli(chn, "frequency", cfg->lo_hz);
	return true;
}

/* simple configuration and streaming */
/* usage:
 * Default context, assuming local IIO devices, i.e., this script is run on ADALM-Pluto for example
 $./a.out
 * URI context, find out the uri by typing `iio_info -s` at the command line of the host PC
 $./a.out usb:x.x.x
 */
int main (int argc, char **argv)
{
    //??????????????????????????????
	double RFfre = MHZ(433);                          //????????????433M
	double gain = 60.0;                                  //??????
    double fskdev = 100000;                            //FSK??????10k
    double hopfre = 1000000;                           //????????????

    //????????????
    long long baseSampleRate = 2500000;                   //???????????????
    double samplenum = 250000;
    double frametime = samplenum/baseSampleRate;
    double digitalrate = 20000;
    double datatime = 1/digitalrate;
    double datanum = frametime/datatime;
    double symnum = samplenum/datanum;

    //????????????
    double per_jump_time = frametime;
    double jump_cycle = 10;
    double jump_index = jump_cycle-1;
    double jump_cycletime = per_jump_time*jump_cycle;

    // Streaming devices?????????
	struct iio_device *rx;  //??????????????????cf-ad9361-lpc

	// RX and TX sample counters   ?????????????????????
	size_t nrx = 0;

	// Stream configurations
	struct stream_cfg rxcfg;    //??????????????????????????????????????????????????????

	// Listen to ctrl+c and IIO_ENSURE
	signal(SIGINT, handle_sig);      //????????????????????????ctrl+c????????????????????????????????????handle_sig

	// RX stream config
	rxcfg.bw_hz = MHZ(0.75);                   // ?????????         ?????????200k 1 56M???
	rxcfg.fs_hz = baseSampleRate;           // ???????????????      ?????? ???2.083333M 1 61.44M???
	rxcfg.lo_hz = RFfre;                    // ????????????       ?????????325M 1 3.8G???
	rxcfg.gain_mode = "manual";        // ??????????????????    ?????????manual fast_attack slow_attack hybrid
	rxcfg.gain  = gain;                     // ??????          ?????????-1 1 73???dB
	rxcfg.rfport = "A_BALANCED";            // port A (select for rf freq.)    ?????????A_BALANCED B_BALANCED C_BALANCED A_N A_P B_N B_P C_N C_P TX_MONITOR1 TX_MONITOR2 TX_MONITOR1_2

	printf("* Acquiring IIO context\n");                                                              //??????PLUTO???context????????????????????????????????????ip????????????
	if (argc == 1) {
		IIO_ENSURE((ctx = iio_create_default_context()) && "No context");
	}
	else if (argc == 2) {
		IIO_ENSURE((ctx = iio_create_context_from_uri(argv[1])) && "No context");
	}
	IIO_ENSURE(iio_context_get_devices_count(ctx) > 0 && "No devices");                                //??????????????????

	printf("* Acquiring AD9361 streaming devices\n");                                                  //???cf-ad9361-lpc????????????????????????rx
	IIO_ENSURE(get_ad9361_stream_dev(RX, &rx) && "No rx dev found");

	printf("* Configuring AD9361 for streaming\n");                                                    //??????cfg????????????????????????ad9361-phy???voltage0??????ad9363??????????????????altvoltage0?????????????????????
	IIO_ENSURE(cfg_ad9361_streaming_ch(&rxcfg, RX, 0) && "RX port 0 not found");

	printf("* Initializing AD9361 IIO streaming channels\n");                                          //???cf-ad9361-lpc????????????????????????rx0_i???rx0_q
	IIO_ENSURE(get_ad9361_stream_ch(RX, rx, 0, &rx0_i) && "RX chan i not found");                      //?????????voltage0???voltage1,????????????????????????altvotage??????
	IIO_ENSURE(get_ad9361_stream_ch(RX, rx, 1, &rx0_q) && "RX chan q not found");

	printf("* Enabling IIO streaming channels\n");                                                      //????????????
	iio_channel_enable(rx0_i);
	iio_channel_enable(rx0_q);

	printf("* Creating non-cyclic IIO buffers with 1 MiS\n");                                           //??????iio buffer
	rxbuf = iio_device_create_buffer(rx, samplenum, false);                                             //???rx ?????? ????????????buffer????????????      ?????????????????????buffer???
	if (!rxbuf) {
		perror("Could not create RX buffer");
		shutdown();
	}

	printf("* Starting IO streaming (press CTRL+C to cancel)\n");

	//????????????????????????????????????
	struct iio_channel *chn = NULL;
    if (!get_lo_chan(RX, &chn)) { return false; }

    //??????????????????????????????
	int hopdev=0;                                           //?????????????????????
	long long catch_gate =702731209814;                     //??????????????????
	int catch_signal = 0;                                   //??????????????????
	double last_signal_rms = 0;                                //???????????????????????????
	int adj_scale = 1;                                     //?????????????????????
	double adj_time = 0.5*frametime/adj_scale;                 //???????????????
	double lock_gate = 1402731209814;                       //????????????
	while (!stop)
	{
        double signal_rms=0;
		ssize_t nbytes_rx, nbytes_tx;                                                                 //long int ??????   ???????????????????????????
		char *p_dat, *p_end;                                                                          //buffer??????
		ptrdiff_t p_inc;                                                                              //??????????????? ??????

        // Refill RX buffer
		nbytes_rx = iio_buffer_refill(rxbuf);                                                         //????????????
		if (nbytes_rx < 0) { printf("Error refilling buf %d\n",(int) nbytes_rx); shutdown(); }

		// READ: Get pointers to RX buf and read IQ from RX buf port 0
		p_inc = iio_buffer_step(rxbuf);                                                               //rxbuffer???????????????????????????
		p_end = iio_buffer_end(rxbuf);                                                                //rxbuffer????????????????????????
		for (p_dat = (char *)iio_buffer_first(rxbuf, rx0_i); p_dat < p_end; p_dat += p_inc) {         //??????????????????????????? ????????????????????????????????????????????????????????????IO????????????
			// Example: swap I and Q
			const int16_t i = ((int16_t*)p_dat)[0]; // Real (I)                                       //
			const int16_t q = ((int16_t*)p_dat)[1]; // Imag (Q)                                       //
			//printf("q = %d,i = %d\n",q,i);
			signal_rms = signal_rms + i*i+q*q;

		}

        printf("\n rms = %f \n",signal_rms);
        //???????????????????????????????????????????????????????????????40%??????
        if(signal_rms<catch_gate)
        {
            catch_signal = 0;
            if (hopdev != 9)
            {
                hopdev++;
            }
            else
            {
                hopdev = 0;
            }
        }
        else
        {
            if(catch_signal == 2)
            {
                last_signal_rms = signal_rms;
            }
            if(catch_signal == 0)
            {
                catch_signal = 2;
            }
            else
            {
                catch_signal = 1;
            }
            printf("catch it\n");
        }

        //?????????????????????????????????????????????????????????????????????????????????90%??????
        if(catch_signal == 1)
        {
             if(signal_rms< lock_gate)
             {
                printf("sleep,now,the lst is %f,now is %f",lock_gate,signal_rms);
                //last_signal_rms = (signal_rms+last_signal_rms)/2;
                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
             }
        }


        //????????????
        if (hopdev != 9)
        {
            hopdev++;
        }
        else
        {
            hopdev = 0;
        }
        wr_ch_lli(chn, "frequency", (long long)(rxcfg.lo_hz+hopdev*hopfre));
		// Sample counter increment and status output
		nrx += nbytes_rx / iio_device_get_sample_size(rx);
		printf("\tRX %8.2f MSmp", nrx/1e6);

	}

	shutdown();

	return 0;
}
