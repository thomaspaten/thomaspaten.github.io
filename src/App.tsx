import React, { useState } from 'react';
import { ChevronRight, Github, Thermometer, Droplets, Wind, Egg, CheckCircle, ExternalLink } from 'lucide-react';

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
    { url: "https://via.placeholder.com/800x600/1a1a2e/eee?text=Incubator+Overview", caption: "Complete incubator system" },
    { url: "https://via.placeholder.com/800x600/16213e/eee?text=Arduino+Setup", caption: "Arduino Uno with sensors" },
    { url: "https://via.placeholder.com/800x600/0f3460/eee?text=Temperature+Control", caption: "Temperature & humidity sensors" },
    { url: "https://via.placeholder.com/800x600/533483/eee?text=Final+Product", caption: "Working prototype with eggs" }
  ];

  const projectSteps: ProjectStep[] = [
    {
      title: "System Design",
      description: "Designed the control system architecture with Arduino Uno as the brain",
      details: "Planned component layout, power requirements, and sensor placement for optimal monitoring",
      icon: <Egg className="w-5 h-5" />
    },
    {
      title: "Temperature Control",
      description: "Implemented PID control for maintaining 37.5°C (99.5°F)",
      details: "Used DHT22 sensor with heating element and relay module for precise temperature regulation",
      icon: <Thermometer className="w-5 h-5" />
    },
    {
      title: "Humidity Management",
      description: "Automated humidity control system maintaining 45-55% RH",
      details: "Integrated ultrasonic humidifier with water level sensor and automatic refill alerts",
      icon: <Droplets className="w-5 h-5" />
    },
    {
      title: "Egg Rotation",
      description: "Built automatic egg turning mechanism with servo motors",
      details: "Programmed to rotate eggs every 2 hours to ensure proper embryo development",
      icon: <Wind className="w-5 h-5" />
    }
  ];

  const techStack: string[] = [
    "Arduino Uno", "C++", "DHT22 Sensor", "Servo Motors", 
    "LCD Display", "Relay Modules", "PID Library"
  ];

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 text-white">
      {/* Hero Section */}
      <div className="container mx-auto px-4 py-16">
        <div className="max-w-6xl mx-auto">
          {/* Project Header */}
          <div className="mb-12 text-center">
            <h1 className="text-5xl font-bold mb-4 bg-gradient-to-r from-amber-400 to-orange-500 bg-clip-text text-transparent">
              Arduino Chick Incubator
            </h1>
            <p className="text-xl text-slate-300 max-w-2xl mx-auto">
              An automated incubation system with precise temperature and humidity control, 
              featuring automatic egg rotation for optimal hatch rates
            </p>
          </div>

          {/* Image Gallery */}
          <div className="mb-16">
            <div className="relative rounded-xl overflow-hidden shadow-2xl bg-slate-800/50 backdrop-blur">
              <img 
                src={projectImages[activeImage].url} 
                alt={projectImages[activeImage].caption}
                className="w-full h-[500px] object-cover"
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
              <p className="text-slate-300">±0.5°C accuracy with PID control algorithm</p>
            </div>
            <div className="bg-slate-800/50 backdrop-blur rounded-xl p-6 border border-slate-700">
              <Droplets className="w-10 h-10 text-blue-400 mb-4" />
              <h3 className="text-xl font-semibold mb-2">Humidity Control</h3>
              <p className="text-slate-300">Maintains optimal 45-55% relative humidity</p>
            </div>
            <div className="bg-slate-800/50 backdrop-blur rounded-xl p-6 border border-slate-700">
              <Wind className="w-10 h-10 text-green-400 mb-4" />
              <h3 className="text-xl font-semibold mb-2">Auto Rotation</h3>
              <p className="text-slate-300">Turns eggs every 2 hours automatically</p>
            </div>
          </div>

          {/* Build Process */}
          <div className="mb-16">
            <h2 className="text-3xl font-bold mb-8 text-center">Build Process</h2>
            <div className="space-y-4">
              {projectSteps.map((step: ProjectStep, idx: number) => (
                <div 
                  key={idx}
                  className="bg-slate-800/50 backdrop-blur rounded-xl border border-slate-700 overflow-hidden"
                >
                  <button
                    onClick={() => setActiveStep(activeStep === idx ? null : idx)}
                    className="w-full p-6 flex items-center justify-between hover:bg-slate-700/30 transition-colors"
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
                  </button>
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
            <div className="flex flex-wrap gap-3 justify-center">
              {techStack.map((tech: string, idx: number) => (
                <span 
                  key={idx}
                  className="px-4 py-2 bg-slate-800/50 backdrop-blur rounded-full border border-slate-700 text-sm"
                >
                  {tech}
                </span>
              ))}
            </div>
          </div>

          {/* Results */}
          <div className="bg-gradient-to-r from-amber-500/10 to-orange-500/10 rounded-xl p-8 border border-amber-500/20 mb-16">
            <h2 className="text-3xl font-bold mb-6 text-center">Project Results</h2>
            <div className="grid md:grid-cols-2 gap-6">
              <div className="flex items-start gap-3">
                <CheckCircle className="w-5 h-5 text-green-400 mt-1 flex-shrink-0" />
                <p className="text-slate-300">Successfully hatched 18 out of 20 eggs (90% hatch rate)</p>
              </div>
              <div className="flex items-start gap-3">
                <CheckCircle className="w-5 h-5 text-green-400 mt-1 flex-shrink-0" />
                <p className="text-slate-300">Total cost under $50 for complete system</p>
              </div>
              <div className="flex items-start gap-3">
                <CheckCircle className="w-5 h-5 text-green-400 mt-1 flex-shrink-0" />
                <p className="text-slate-300">21-day fully automated incubation cycle</p>
              </div>
              <div className="flex items-start gap-3">
                <CheckCircle className="w-5 h-5 text-green-400 mt-1 flex-shrink-0" />
                <p className="text-slate-300">LCD display for real-time monitoring</p>
              </div>
            </div>
          </div>

          {/* CTA Buttons */}
          <div className="flex gap-4 justify-center">
            <a 
              href="https://github.com/yourusername/arduino-incubator" 
              className="flex items-center gap-2 px-6 py-3 bg-slate-800 hover:bg-slate-700 rounded-lg transition-colors"
            >
              <Github className="w-5 h-5" />
              View on GitHub
            </a>
            <a 
              href="#" 
              className="flex items-center gap-2 px-6 py-3 bg-gradient-to-r from-amber-500 to-orange-500 hover:from-amber-400 hover:to-orange-400 rounded-lg transition-colors"
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