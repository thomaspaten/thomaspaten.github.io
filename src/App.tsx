import React, { useState } from 'react';
import { 
  ChevronRight, Github, Thermometer, Droplets, Egg, 
  CheckCircle, ExternalLink, Mail, Linkedin, RefreshCcwDot,
  Cpu, Code, Box,
  Wind, Shield, Bluetooth, ToggleLeft,
  Flame
} from 'lucide-react';
import fritzingWiringPlanImg from '../public/img/fritzing-wiring-plan.png'
import arduinoCircuitImg from '../public/img/arduino-circuit.png'
import bottomSectionImg from '../public/img/bottom-section.jpg'
import topSectionImg from '../public/img/top-section.jpg'
import hatchedChickImg from '../public/img/hatched-chick.jpg'
import linkedinProfileImg from '../public/img/linkedin-profile.png'

interface ProjectImage {
  url: string;
  caption: string;
}

interface ProjectStep {
  title: string;
  description: string;
  details: string;
  icon: React.ReactNode;
}

const IncubatorPortfolio: React.FC = () => {
  const [activeImage, setActiveImage] = useState<number>(0);
  const [activeStep, setActiveStep] = useState<number | null>(null);

  // Replace these with your actual image URLs
  const projectImages: ProjectImage[] = [
    { url: fritzingWiringPlanImg, caption: "Fritzing Wiring Plan: Complete Incubator Circuit" },
    { url: arduinoCircuitImg, caption: "Wired Incubator: From Fritzing to Reality" },
    { url: bottomSectionImg, caption: "Setter Phase: Days 1–18 – Embryo Development (Testing Phase)" },
    { url: topSectionImg, caption: "Hatching Phase: Days 18–21 – Chicks Emerge (Testing Phase)" },
    { url: hatchedChickImg, caption: "Hatched Chicks: First Moments of Life 🐣" },
  ];

const projectSteps: ProjectStep[] = [
  {
    title: "System Design",
    description: "Advanced environmental control system powered by Arduino Mega",
    details: "To support the continuous egg-laying cycle of hens, we developed an environmental control system capable of incubating up to 120 new eggs every 18 days in a setter. This system maintains optimal conditions for egg incubation across three zones—Setter, Hatcher, and Exterior—using precise temperature and humidity regulation. Sensors monitor temperature and humidity levels, connected to an Arduino Mega via a multiplexer to manage multiple sensors on a single I2C bus, overcoming the microcontroller’s limited I2C pin availability. Dimmable lights adjust brightness based on temperature readings, while five PWM-controlled fans regulate airflow, with TACH pins providing real-time fan speed monitoring. A linear actuator, driven by a motor driver, operates in timed cycles to gently rotate eggs, ensuring uniform development. A watchdog timer resets the Arduino in case of a system crash, ensuring reliability. The system dynamically adjusts fan speeds and light intensity based on temperature and humidity data from the three zones to maintain ideal incubation conditions.",
    icon: <Egg className="w-5 h-5" />
  },
  {
    title: "Temperature, Humidity Management",
    description: "Dynamic threshold adjustments.",
    details: "The climate control mechanism monitors temperature and humidity in the Hatcher, Setter, and exterior regions using AHT20 sensors, adjusting dimmable lights (0–80% brightness) and fans (0–100% speed) based on predefined thresholds to maintain optimal conditions, such as 37.2–37.5°C and 65–75% humidity for the Hatcher, and 37.5°C and 50–55% for the Setter. These thresholds account for system inertia, such as residual heat from lightbulbs, ensuring precise temperature and humidity regulation, with external temperature influencing Setter dimmer settings. Fallback values and sensor health checks enhance reliability by mitigating potential sensor failures.",
    icon: <Thermometer className="w-5 h-5" />
  },
  {
    title: "Egg Rotation",
    description: "Automated egg turning with linear actuator",
    details: "The linear actuator in the egg incubator system gently turns eggs to prevent the embryo from sticking to the inner egg wall. It uses a motor driver, controlled by specific pins, to move back and forth. The actuator follows a fixed sequence: it retracts for 60 seconds, stops for 10 seconds, then repeats 11 cycles of 3-second extensions and 3-second retractions, each followed by a 13-minute pause. One complete cycle, from start to reset, takes about 4 hours and 48 minutes.",    
    icon: <RefreshCcwDot className="w-5 h-5" />
  },
  {
    title: "System Reliability",
    description: "Stable and continuous operation to protect delicate bird embryos through safeguards",
    details:"The incubator system uses several features to stay reliable. A watchdog timer resets the Arduino if it freezes, ensuring it keeps running. Sensors are checked multiple times for accurate data, and if they fail, the system uses backup values to keep going. The I2C communication system retries connections and resets if needed to avoid lockups. Fans and lights are precisely controlled based on temperature and humidity, with fan speeds monitored to catch issues. The actuator follows a strict sequence with timed movements and resets after completing its cycles. Debug messages help track problems without slowing the system down. These mechanisms help maintain smooth incubator operation, protecting the delicate lives of developing birds despite potential hardware or software failures.",
    icon: <Shield className="w-5 h-5" />
  }
];

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 text-white">
      {/* Hero Section */}
      <div className="container mx-auto px-4 py-16">
        <div className="max-w-6xl mx-auto">
          {/* Project Header */}
          <div className="mb-12 text-center">
            <h1 className="text-5xl font-bold mb-4 bg-gradient-to-r from-amber-400 to-orange-500 bg-clip-text text-transparent">
              Nurturing Life: Arduino-Powered Poultry Incubator
            </h1>
            <p className="text-xl text-slate-300 max-w-2xl mx-auto">
              An automated incubation system with precise temperature and humidity control, 
              featuring automatic egg rotation for optimal hatch rates
            </p>
          </div>

          {/* Image Gallery */}
          <div className="mb-16">
            <div className=" rounded-xl overflow-hidden shadow-2xl bg-slate-800/50 backdrop-blur w-auto md:w-[500px] mx-auto">
              <img 
                src={projectImages[activeImage].url} 
                alt={projectImages[activeImage].caption}
                className="w-auto h-full object-cover"
              />
              <div className="absolute bottom-0 left-0 right-0 bg-gradient-to-t from-black/80 to-transparent p-6">
                <p className="text-lg font-medium">{projectImages[activeImage].caption}</p>
              </div>
            </div>
            
            {/* Thumbnail Navigation */}
            <div className="flex gap-4 mt-6 justify-center">
              {projectImages.map((img: ProjectImage, idx: number) => (
                <button
                  key={idx}
                  onClick={() => setActiveImage(idx)}
                  className={`relative w-24 h-20 rounded-lg overflow-hidden transition-all ${
                    activeImage === idx 
                      ? 'ring-2 ring-amber-400 scale-105' 
                      : 'opacity-70 hover:opacity-100'
                  }`}
                >
                  <img src={img.url} alt="" className="w-full h-full object-cover" />
                </button>
              ))}
            </div>
          </div>

          {/* Key Features */}
          <div className="grid md:grid-cols-3 gap-6 mb-16">
            <div className="bg-slate-800/50 backdrop-blur rounded-xl p-6 border border-slate-700">
              <Thermometer className="w-10 h-10 text-amber-400 mb-4" />
              <h3 className="text-xl font-semibold mb-2">Precise Temperature</h3>
              <p className="text-slate-300">Target: Setter 37°C, Hatcher 37.5°C <br/> Accuracy: ±0.3°C</p>
            </div>
            <div className="bg-slate-800/50 backdrop-blur rounded-xl p-6 border border-slate-700">
              <Droplets className="w-10 h-10 text-blue-400 mb-4" />
              <h3 className="text-xl font-semibold mb-2">Adaptative Humidity</h3>
              <p className="text-slate-300">Regulates 50-55% Setter, 65-75% Hatching with Fans</p>
            </div>

            <div className="bg-slate-800/50 backdrop-blur rounded-xl p-6 border border-slate-700">
              <RefreshCcwDot className="w-10 h-10 text-green-400 mb-4" />
              <h3 className="text-xl font-semibold mb-2">Auto Rotation</h3>
              <p className="text-slate-300">Automatically rotates eggs every 4h 46m</p>
            </div>
          </div>

          {/* Build Process */}
          <div className="mb-16">
            <h2 className="text-3xl font-bold mb-8 text-center">Build Process</h2>
           
            <div className="space-y-4">
              {projectSteps.map((step: ProjectStep, idx: number) => (
                <div 
                  key={idx}
                  className="bg-slate-800/50 backdrop-blur rounded-xl border border-slate-700 overflow-hidden hover:bg-slate-700/30 transition-colors cursor-pointer"
                >   
                  <div
                  role="button"
  tabIndex={0}
    onKeyDown={(e) => e.key === 'Enter' && setActiveStep(activeStep === idx ? null : idx)}
                    onClick={() => setActiveStep(activeStep === idx ? null : idx)}
                    className=" w-full p-6 flex items-center justify-between !outline-none]"
                  >
                    <div className="flex items-center gap-4">
                      <div className="w-12 h-12 rounded-full bg-gradient-to-br from-amber-400 to-orange-500 flex items-center justify-center">
                        {step.icon}
                      </div>
                      <div className="text-left">
                        <h3 className="text-xl font-semibold">{step.title}</h3>
                        <p className="text-slate-400">{step.description}</p>
                      </div>
                    </div>
                    <ChevronRight className={`w-5 h-5 transition-transform ${activeStep === idx ? 'rotate-90' : ''}`} />
                  </div>

                  {activeStep === idx && (
                    <div className="px-6 pb-6 pt-2">
                      <p className="text-slate-300 pl-16">{step.details}</p>
                    </div>
                  )}
                </div>
              ))}
            </div>
          </div>

          {/* Tech Stack */}
<div className="mb-16">
  <h2 className="text-3xl font-bold mb-8 text-center">Technologies Used</h2>
  
  <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-4">

    {/* Programming */}
    <div className="bg-slate-800/50 backdrop-blur rounded-lg p-4 border border-slate-700">
      <div className="flex items-start gap-3">
        <Code className="w-5 h-5 text-amber-400 mt-1 flex-shrink-0" />
        <div>
          <h4 className="font-semibold text-white">Programming Language</h4>
          <p className="text-sm text-slate-400">C++</p>
        </div>
      </div>
    </div>

    {/* Environmental Chamber */}
    <div className="bg-slate-800/50 backdrop-blur rounded-lg p-4 border border-slate-700 md:col-span-2 lg:col-span-1">
      <div className="flex items-start gap-3">
        <Box className="w-5 h-5 text-amber-400 mt-1 flex-shrink-0" />
        <div>
          <h4 className="font-semibold text-white">Environmental Chamber</h4>
          <p className="text-sm text-slate-400">Repurposed Refrigerator Housing</p>
        </div>
      </div>
    </div>

    {/* Microcontroller */}
    <div className="bg-slate-800/50 backdrop-blur rounded-lg p-4 border border-slate-700">
      <div className="flex items-start gap-3">
        <Cpu className="w-5 h-5 text-amber-400 mt-1 flex-shrink-0" />
        <div>
          <h4 className="font-semibold text-white">Microcontroller</h4>
          <p className="text-sm text-slate-400">Arduino Mega</p>
        </div>
      </div>
    </div>

    {/* Sensors */}
    <div className="bg-slate-800/50 backdrop-blur rounded-lg p-4 border border-slate-700">
      <div className="flex items-start gap-3">
        <Thermometer className="w-5 h-5 text-amber-400 mt-1 flex-shrink-0" />
        <div>
          <h4 className="font-semibold text-white">Temperature & Humidity Sensors</h4>
          <p className="text-sm text-slate-400">TCA9548A Multiplexer, AHT20</p>
        </div>
      </div>
    </div>

    {/* Heat System */}
    <div className="bg-slate-800/50 backdrop-blur rounded-lg p-4 border border-slate-700">
      <div className="flex items-start gap-3">
        <Flame className="w-5 h-5 text-amber-400 mt-1 flex-shrink-0" />
        <div>
          <h4 className="font-semibold text-white">Heat System</h4>
          <p className="text-sm text-slate-400">AC Dimmers, 100W Incandescent Light Bulb</p>
        </div>
      </div>
    </div>

    {/* Egg Turner */}
    <div className="bg-slate-800/50 backdrop-blur rounded-lg p-4 border border-slate-700">
      <div className="flex items-start gap-3">
        <RefreshCcwDot  className="w-5 h-5 text-amber-400 mt-1 flex-shrink-0" />
        <div>
          <h4 className="font-semibold text-white">Egg Turner</h4>
          <p className="text-sm text-slate-400">Double BTS7960 Driver, Linear Actuator 1500N</p>
        </div>
      </div>
    </div>

    {/* Ventilation System */}
    <div className="bg-slate-800/50 backdrop-blur rounded-lg p-4 border border-slate-700">
      <div className="flex items-start gap-3">
        <Wind className="w-5 h-5 text-amber-400 mt-1 flex-shrink-0" />
        <div>
          <h4 className="font-semibold text-white">Ventilation System</h4>
          <p className="text-sm text-slate-400">ARCTIC F8 PWM, ARCTIC P14 PWM PST</p>
        </div>
      </div>
    </div>



  </div>


</div>


          {/* Results */}
          <div className="bg-gradient-to-r from-amber-500/10 to-orange-500/10 rounded-xl p-8 border border-amber-500/20 mb-16">
            <h2 className="text-3xl font-bold mb-6 text-center">Project Results</h2>
            <div className="grid md:grid-cols-2 gap-6">
              <div className="flex items-start gap-3">
                <CheckCircle className="w-5 h-5 text-green-400 mt-1 flex-shrink-0" />
                <p className="text-slate-300">Successfully hatched 11 out of 12 eggs on second try</p>
              </div>
              <div className="flex items-start gap-3">
                <CheckCircle className="w-5 h-5 text-green-400 mt-1 flex-shrink-0" />
                <p className="text-slate-300">Total cost under $350 for complete system</p>
              </div>
              <div className="flex items-start gap-3">
                <CheckCircle className="w-5 h-5 text-green-400 mt-1 flex-shrink-0" />
                <p className="text-slate-300">21-day fully automated incubation cycle</p>
              </div>
              <div className="flex items-start gap-3">
                <CheckCircle className="w-5 h-5 text-green-400 mt-1 flex-shrink-0" />
                <p className="text-slate-300">Sustainable Source of Delight</p>
              </div>
            </div>
          </div>


{/* Projected Improvements - Add this after the Results section */}
<div className="mb-16">
  <h2 className="text-3xl font-bold mb-8 text-center">Projected Improvements</h2>
  
  <div className="grid md:grid-cols-3 gap-6">
    {/* Bluetooth Interface */}
    <div className="bg-slate-800/50 backdrop-blur rounded-xl p-6 border border-slate-700">
      <div className="flex items-start gap-4">
        <div className="w-12 h-12 rounded-full bg-gradient-to-br from-blue-400 to-blue-600 flex items-center justify-center flex-shrink-0">
          <Bluetooth className="w-6 h-6 text-white" />
        </div>
        <div>
          <h3 className="text-xl font-semibold mb-2">Bluetooth Monitoring</h3>
          <p className="text-slate-400">Wireless serial monitoring without physical connection</p>
        </div>
      </div>
    </div>

    {/* Section Control */}
    <div className="bg-slate-800/50 backdrop-blur rounded-xl p-6 border border-slate-700">
      <div className="flex items-start gap-4">
        <div className="w-12 h-12 rounded-full bg-gradient-to-br from-green-400 to-green-600 flex items-center justify-center flex-shrink-0">
          <ToggleLeft className="w-6 h-6 text-white" />
        </div>
        <div>
          <h3 className="text-xl font-semibold mb-2">Zone Control</h3>
          <p className="text-slate-400">Independent On/Off switches for setter and hatcher sections</p>
        </div>
      </div>
    </div>

    {/* Egg Turner */}
    <div className="bg-slate-800/50 backdrop-blur rounded-xl p-6 border border-slate-700">
      <div className="flex items-start gap-4">
        <div className="w-12 h-12 rounded-full bg-gradient-to-br from-amber-400 to-orange-500 flex items-center justify-center flex-shrink-0">
          <RefreshCcwDot className="w-5 h-5" />
        </div>
        <div>
          <h3 className="text-xl font-semibold mb-2">Egg Turner</h3>
          <p className="text-slate-400">Building a custom egg turner with a 360-egg capacity</p>
        </div>
      </div>
    </div>
  </div>
</div>

{/* Contact Section */}
<div className="bg-slate-800/50 backdrop-blur rounded-xl p-8 border border-slate-700 mb-16">
  <div className="flex flex-col md:flex-row items-center gap-8">
    {/* Profile Picture */}
    <div className="flex-shrink-0">
      <img 
        src={linkedinProfileImg} 
        alt="Thomas Patenaude Poulin"
        className="w-32 h-32 md:w-40 md:h-40 rounded-full border-4 border-amber-400/30 shadow-xl"
      />
    </div>
    
    {/* Contact Info */}
    <div className="flex-1 text-center md:text-left">
      <h2 className="text-3xl font-bold mb-2">Let's Connect!</h2>
      <p className="text-xl text-amber-400 mb-4">Thomas Patenaude Poulin</p>
      <p className="text-slate-300 mb-6">
Enthusiastic developer with a strong foundation in full-stack TypeScript and Node.js, passionate about exploring embedded systems to craft innovative solutions. 
Open to new projects and collaboration opportunities in both domains.
      </p>
      <div className="flex flex-wrap gap-4 justify-center md:justify-start">
        <a 
          href="mailto:your.email@example.com" 
          className="flex items-center gap-2 px-6 py-3 bg-amber-500 hover:bg-amber-400 rounded-lg transition-colors !text-white"
        >
          <Mail className="w-5 h-5" />
          Email Me
        </a>
        <a 
          href="https://www.linkedin.com/in/thomas-patenaude-poulin/" 
          className="flex items-center gap-2 px-6 py-3 bg-slate-700 hover:bg-slate-600 rounded-lg transition-colors !text-white"
        >
          <Linkedin className="w-5 h-5" />
          LinkedIn
        </a>
      </div>
    </div>
  </div>
</div>


          {/* CTA Buttons */}
          <div className="flex gap-4 justify-center">
            <a 
              href="https://github.com/yourusername/arduino-incubator" 
              className="flex items-center gap-2 px-6 py-3 bg-slate-800 hover:bg-slate-700 rounded-lg transition-colors !text-white"
            >
              <Github className="w-5 h-5" />
              View on GitHub
            </a>
            <a 
              href="#" 
              className="flex items-center gap-2 px-6 py-3 bg-gradient-to-r from-amber-500 to-orange-500 hover:from-amber-400 hover:to-orange-400 rounded-lg transition-colors !text-white"
            >
              <ExternalLink className="w-5 h-5" />
              Live Documentation
            </a>
          </div>
        </div>
      </div>
    </div>
  );
};

export default IncubatorPortfolio;