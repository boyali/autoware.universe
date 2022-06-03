//
// Created by ali on 20/05/22.
//

#include "delay_observer.hpp"

DelayObserver::DelayObserver(s_filter_data const& Qfilter_data, s_model_G_data const& Gdata) :
		q_cut_off_frequency_{ Qfilter_data.cut_off_frq },
		q_time_constant_tau_{ Qfilter_data.time_constant },
		Qfilter_tf_(Qfilter_data.TF),
		num_den_coeff_names_{ Gdata.num_den_coeff_names },
		m_num_factor_{ Gdata.num_coeff_ },
		m_den_factor_{ Gdata.den_coeff_ },
		G_(Gdata.TF)
{

}
