#ifndef RANGING_ENGINE_H
#define RANGING_ENGINE_H

#include <stdint.h>
#include <zephyr/drivers/ieee802154/dw1000.h>

enum extraction_mode {
  EXTRACT_LOCAL_ONLY,
  EXTRACT_LOCAL_BIDIRECTIONAL,
  EXTRACT_ALL
};

/* all these data structures are expected to be only be relevant for the duration of a ranging round
 * therefore the additional space we have to spent for associating each item with the pair of nodes
 * is assumed to be negligible.
 */
struct twr_template
{
    uint8_t ranging_initiator_id, ranging_responder_id;
    uint64_t tx_init, rx_init, tx_resp, rx_resp, tx_final, rx_final;

    // these are only relevant if ranging_initiator_id and ranging_responder_id are both not our own ranging id
    uint64_t local_rx_init, local_rx_resp, local_rx_final;
};

struct freq_offset
{
    uint8_t from_id, to_id;
    float offset;
};

struct measurement
{
    uint8_t ranging_initiator_id, ranging_responder_id;
    float tof, tdoa;
    float quality; // currently unused but useful in the future
};

int32_t compute_prop_time(int32_t initiator_roundtrip, int32_t initiator_reply,
			  int32_t replier_roundtrip, int32_t replier_reply);
float time_to_dist(float tof);
float calculate_propagation_time_alternative(uint64_t tx_init, uint64_t rx_init, uint64_t tx_resp,
					     uint64_t rx_resp, uint64_t tx_final,
					     uint64_t rx_final);

int fill_twr_template_from_frames(const struct dwt_ranging_frame_info *frame_infos,
				  uint8_t initiator_id, int round_length, int phases,
				  struct twr_template *twr_templates, int with_reject);

int have_freq_offset(struct freq_offset *offsets, int offsets_length, uint8_t from_id,
		     uint8_t to_id);

int frequencies_from_cfos(const struct dwt_ranging_frame_info *frame_infos, int round_length,
			  int phases, struct freq_offset *offsets);

int extract_round_templates(const struct dwt_ranging_frame_info *frame_infos,
			    const struct mtm_ranging_config *conf, enum extraction_mode mode);

int calculate_measurements(const struct twr_template *twr_templates, int template_count,
			   const struct freq_offset *offsets, int frequency_count,
			   struct measurement *measurements);

int calculate_frequency_offsets(const struct twr_template *twr_templates, int template_count,
				struct freq_offset *offsets);


#endif
