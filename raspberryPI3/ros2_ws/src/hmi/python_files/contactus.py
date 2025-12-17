import streamlit as st


def generate_contact():

	HOME_CSS = f"""
	<style>
	body, .stApp {{
	margin: 0;
	padding: 0;
	height: auto;
	overflow: auto;
	background: #000;
	}}


	.block-container {{
	padding: 0 !important;
	margin: 0 !important;
	}}
	
	.home-title {{
	font-size:90px;
	color:#C0B0FF;
	font-weight:900;
	text-shadow: 0 0 0px rgba(0,191,255,0.8), 0 0 px rgba(0,128,255,0.6);
	margin-top:300px;
	margin-left:800px;
	}}

	.home-sub {{
	font-size:22px;
	color:#e0f5ff;
	margin-top:0px;
	margin-left:850px;

	text-shadow:0 0 10px rgba(0,150,255,0.7);
	}}

	@keyframes fadeIn {{
	0% {{ opacity: 0; transform: scale(1.05); }}
	100% {{ opacity: 1; transform: scale(1); }}
	}}

	.contact-form {{
		width: 40%;
		margin: -350px 0px 0px 100px;
		padding: 30px;
		background: rgba(255,255,255,0.08);
		border-radius: 16px;
		backdrop-filter: blur(8px);
		box-shadow: 0 0 25px rgba(0,0,0,0.4);
	}}

	.contact-form label {{
		color: #E8E2FF;
		font-size: 20px;
		font-weight: 600;
	}}

	input[type="text"], input[type="email"], textarea {{
		width: 100%;
		padding: 14px;
		margin-top: 8px;
		margin-bottom: 22px;
		border-radius: 10px;
		border: none;
		background: rgba(255,255,255,0.18);
		color: white;
		font-size: 18px;
	}}

	textarea {{
		height: 180px;
		resize: none;
	}}
	button {{
    padding: 12px 20px;
    font-size: 18px;
    font-weight: bold;
    color: white;
    background-color: #4B39EF;
    border: none;
    border-radius: 10px;
    cursor: pointer;
	}}

	.social-icons {{
    margin-top: -250px;
	margin-left : 950px;
    display: flex;
    gap: 25px;
	}}

	.social-icons a {{
		font-size: 36px;
		color: #E8E2FF;
		text-decoration: none;
		transition: 0.3s;
	}}

	.social-icons a:hover {{
		color: #9D7CFF;
		text-shadow: 0 0 12px #A277FF, 0 0 24px #8C5CFF;
		transform: scale(1.15);
	}}
	</style>
	"""
	st.markdown(HOME_CSS, unsafe_allow_html=True)
	st.markdown(f"""
		<div class="home-title">Get in touch</div>
		<div class="home-sub">We're here to help you! How can we help?</div>
	""", unsafe_allow_html=True)

	st.markdown("""
	<style>

	html, body, [data-testid="stAppViewContainer"] {
		margin: 0;
		padding: 0;
		height: 100%;
	}

	[data-testid="stAppViewContainer"] {
		background: linear-gradient(135deg, #0d0a36, #0b2555, #3b003a);
		background-size: 400% 400%;
		animation: gradientFade 6s ease infinite;
	}

	@keyframes gradientFade {
		0%   { background-position: 0% 50%; }
		50%  { background-position: 100% 50%; }
		100% { background-position: 0% 50%; }
	}

	</style>
	""", unsafe_allow_html=True)

	st.markdown("""
	<div class="contact-form">
		<form action="https://formspree.io/f/myzaekgg" method="POST" target="_blank">
        <label>Full Name</label>
        <input type="text" name="name" placeholder="Your full name">
        <label>Email Address</label>
        <input type="email" name="email" placeholder="your@email.com">
        <label>Your Message</label>
        <textarea name="message" placeholder="Write your message here..."></textarea>
        <button type="submit">Send Message</button>
    </form>
	</div>
	""", unsafe_allow_html=True)

	st.markdown("""
	<div class="social-icons">
    <a href="https://www.instagram.com/your_profile" target="_blank">
        <img src="https://cdn-icons-png.flaticon.com/512/174/174855.png" width="32">
    </a>
    <a href="https://twitter.com/your_profile" target="_blank">
        <img src="https://cdn-icons-png.flaticon.com/512/733/733579.png" width="32">
    </a>
    <a href="https://www.linkedin.com/in/your_profile/" target="_blank" width="32">
        <img src="https://cdn-icons-png.flaticon.com/512/174/174857.png" width="32">
    </a>
    <a href="mailto:naboulsi@insa-toulouse.fr">
        <img src="https://cdn-icons-png.flaticon.com/512/732/732200.png" width="32">
    </a>
	</div>
	""", unsafe_allow_html=True)