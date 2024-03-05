#![no_std]
#![no_main]

use byte::BytesExt;
use defmt;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_nrf::{
    bind_interrupts, peripherals, radio,
};
use embassy_time::{Duration, Ticker, Timer};
use ieee802154::mac;
use {defmt_rtt as _, panic_probe as _};

/// Create a beacon request frame
pub fn new_beacon_request<'p>(
    sequence: u8,
    version: mac::FrameVersion,
) -> mac::Frame<'p> {
    mac::Frame {
        header: mac::Header {
            seq: sequence,
            frame_type: mac::FrameType::MacCommand,
            auxiliary_security_header: None,
            frame_pending: false,
            ack_request: false,
            pan_id_compress: false,
            version,
            ie_present: false,
            seq_no_suppress: false,
            destination: mac::Address::broadcast(
                &mac::AddressMode::Short,
            ),
            source: None,
        },
        content: mac::FrameContent::Command(mac::command::Command::BeaconRequest),
        payload: &[],
        footer: [0u8; 2],
    }
}


bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;

    let p = embassy_nrf::init(config);

    defmt::info!("Active Scan");

    let mut radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);

    let mut ticker = Ticker::every(Duration::from_secs(2));

    let mut tx_packet = radio::ieee802154::Packet::new();
    let mut sequence = 0;
    const FRAME_VERSION: mac::FrameVersion = mac::FrameVersion::Ieee802154_2003;

    const CHANNELS: [u8; 16] = [
        11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
    ];
    let mut channel_index = 0;
    let mut channel = CHANNELS[channel_index];

    loop {
        let mut rx_packet = radio::ieee802154::Packet::new();
        {
            match select(radio.receive(&mut rx_packet), ticker.next()).await {
                Either::First(result) => {
                    match result {
                        Ok(()) => {
                            let payload: &[u8] = &rx_packet;
                            match payload.read_with::<mac::Frame>(&mut 0, mac::FooterMode::None) {
                                Ok(frame) => {
                                    match frame.content {
                                        mac::FrameContent::Beacon(ref beacon) => {
                                            let coordinator =
                                                if beacon.superframe_spec.pan_coordinator {
                                                    "Coordinator"
                                                } else {
                                                    ""
                                                };
                                            let join = if beacon.superframe_spec.association_permit
                                            {
                                                "Join"
                                            } else {
                                                ""
                                            };
                                            if let Some(ref address) = frame.header.source {
                                                match address {
                                                    mac::Address::Short(
                                                        ref pan_id,
                                                        ref address,
                                                    ) => {
                                                        defmt::info!("CH {} BEACON {=u16:04x}:{=u16:04x} {} {}", channel, pan_id.0, address.0, coordinator, join);
                                                    }
                                                    mac::Address::Extended(
                                                        ref pan_id,
                                                        ref address,
                                                    ) => {
                                                        defmt::info!("CH {} BEACON {=u16:04x}:{=u64:016x} {} {}", channel, pan_id.0, address.0, coordinator, join);
                                                    }
                                                }
                                            }
                                        }
                                        _ => (),
                                    }
                                }
                                Err(_) => {
                                    defmt::error!("Failed to parse frame, {=[u8]:02x}", &rx_packet);
                                }
                            }
                        }
                        Err(error) => {
                            match error {
                                radio::Error::CrcFailed(crc) => {
                                    defmt::error!("Invalid CRC {=u16:04x}", crc);
                                },
                                _ => defmt::error!("Receive failed {}", error),
                            }
                        }
                    }
                }
                Either::Second(_) => {
                    channel = CHANNELS[channel_index];
                    {
                        let beacon_request =
                            new_beacon_request(sequence, FRAME_VERSION);
                        tx_packet.set_len(radio::ieee802154::Packet::CAPACITY);
                        let payload: &mut [u8] = tx_packet.as_mut();
                        let mut offset = 0;
                        let _ = payload.write_with(
                            &mut offset,
                            beacon_request,
                            &mut mac::FrameSerDesContext::no_security(mac::FooterMode::None),
                        );
                        tx_packet.set_len(offset as u8);
                    }
                    radio.set_channel(channel);
                    match radio.try_send(&mut tx_packet).await {
                        Ok(()) => {
                            defmt::info!("CH {} BEACON REQUEST", channel);
                        }
                        Err(ref error) => {
                            defmt::error!("TX failed {}", error);
                        }
                    }
                    sequence = sequence.wrapping_add(1);
                    channel_index += 1;
                    if channel_index >= CHANNELS.len() {
                        channel_index = 0;
                    }
                }
            }
        }
        Timer::after_micros(100).await;
    }
}
