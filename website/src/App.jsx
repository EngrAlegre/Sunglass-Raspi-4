import { useEffect, useState } from 'react';
import { ref, onValue } from 'firebase/database';
import { db } from './firebase';

const getDistanceColor = (cm) => {
  if (!cm || cm === 0) return 'text-gray-400';
  if (cm <= 20) return 'text-red-500';
  if (cm <= 50) return 'text-orange-500';
  if (cm <= 100) return 'text-yellow-500';
  return 'text-green-500';
};

const getBarWidth = (cm) => {
  if (!cm || cm === 0) return 0;
  return Math.min(100, (cm / 200) * 100);
};

const getBarColor = (cm) => {
  if (!cm || cm === 0) return 'bg-gray-200';
  if (cm <= 20) return 'bg-red-500';
  if (cm <= 50) return 'bg-orange-500';
  if (cm <= 100) return 'bg-yellow-500';
  return 'bg-green-500';
};

const patternInfo = (pattern) => {
  switch (pattern) {
    case 'Continuous': return { text: 'Continuous', color: 'bg-red-500', ring: 'ring-red-300' };
    case 'Fast': return { text: 'Fast Pulse', color: 'bg-orange-500', ring: 'ring-orange-300' };
    case 'Slow': return { text: 'Slow Pulse', color: 'bg-yellow-500', ring: 'ring-yellow-300' };
    default: return { text: 'Off', color: 'bg-gray-400', ring: 'ring-gray-200' };
  }
};

const modeInfo = (mode) => {
  switch (mode) {
    case 'Navigation': return { text: 'Navigation', icon: 'bg-blue-500', ring: 'ring-blue-300' };
    case 'Music':      return { text: 'Music', icon: 'bg-purple-500', ring: 'ring-purple-300' };
    case 'Both':       return { text: 'Both', icon: 'bg-emerald-500', ring: 'ring-emerald-300' };
    default:           return { text: 'Unknown', icon: 'bg-gray-400', ring: 'ring-gray-200' };
  }
};

function SensorCard({ label, value }) {
  return (
    <div className="bg-white rounded-xl shadow-sm border border-gray-100 p-5">
      <div className="text-xs font-medium text-gray-500 uppercase tracking-wider">{label}</div>
      <div className={`text-3xl font-bold mt-1 ${getDistanceColor(value)}`}>
        {value > 0 ? value : '--'}
        <span className="text-sm font-normal text-gray-400 ml-1">cm</span>
      </div>
      <div className="mt-2 h-2 bg-gray-100 rounded-full overflow-hidden">
        <div
          className={`h-full rounded-full transition-all duration-300 ${getBarColor(value)}`}
          style={{ width: `${getBarWidth(value)}%` }}
        />
      </div>
    </div>
  );
}

export default function App() {
  const [data, setData] = useState(null);
  const [lastUpdate, setLastUpdate] = useState(null);
  const [now, setNow] = useState(Date.now());

  useEffect(() => {
    const timer = setInterval(() => setNow(Date.now()), 2000);
    return () => clearInterval(timer);
  }, []);

  useEffect(() => {
    const deviceRef = ref(db, 'device');
    return onValue(deviceRef, (snapshot) => {
      const val = snapshot.val();
      if (val) {
        setData(val);
        setLastUpdate(Date.now());
      }
    });
  }, []);

  const isOnline = lastUpdate && (now - lastUpdate) < 10000;
  const pat = patternInfo(data?.pattern);
  const mode = modeInfo(data?.mode);

  const timeAgo = () => {
    if (!lastUpdate) return 'Never';
    const diff = Math.floor((now - lastUpdate) / 1000);
    if (diff < 5) return 'Just now';
    if (diff < 60) return `${diff}s ago`;
    if (diff < 3600) return `${Math.floor(diff / 60)}m ago`;
    return `${Math.floor(diff / 3600)}h ago`;
  };

  return (
    <div className="min-h-screen bg-gray-50">
      <header className="bg-slate-800 text-white">
        <div className="max-w-3xl mx-auto px-4 py-4 flex items-center justify-between">
          <h1 className="text-lg font-semibold tracking-tight">Navigational Sunglasses</h1>
          <div className="flex items-center gap-2 text-sm">
            <span className={`w-2 h-2 rounded-full ${isOnline ? 'bg-green-400 animate-pulse' : 'bg-gray-500'}`} />
            <span className="text-gray-300">{isOnline ? 'Online' : 'Offline'}</span>
          </div>
        </div>
      </header>

      <main className="max-w-3xl mx-auto px-4 py-6 space-y-5">
        <div className="grid grid-cols-3 gap-3">
          <SensorCard label="Left" value={data?.left_cm ?? 0} />
          <SensorCard label="Center" value={data?.center_cm ?? 0} />
          <SensorCard label="Right" value={data?.right_cm ?? 0} />
        </div>

        <div className="grid grid-cols-3 gap-3">
          <div className="bg-white rounded-xl shadow-sm border border-gray-100 p-5">
            <div className="text-xs font-medium text-gray-500 uppercase tracking-wider">Closest Object</div>
            <div className={`text-4xl font-bold mt-1 ${getDistanceColor(data?.min_cm)}`}>
              {data?.min_cm > 0 ? data.min_cm : '--'}
              <span className="text-base font-normal text-gray-400 ml-1">cm</span>
            </div>
          </div>
          <div className="bg-white rounded-xl shadow-sm border border-gray-100 p-5">
            <div className="text-xs font-medium text-gray-500 uppercase tracking-wider">Vibration</div>
            <div className="mt-2 flex items-center gap-3">
              <span className={`w-4 h-4 rounded-full ${pat.color} ring-4 ${pat.ring}`} />
              <span className="text-xl font-semibold text-gray-700">{pat.text}</span>
            </div>
          </div>
          <div className="bg-white rounded-xl shadow-sm border border-gray-100 p-5">
            <div className="text-xs font-medium text-gray-500 uppercase tracking-wider">Mode</div>
            <div className="mt-2 flex items-center gap-3">
              <span className={`w-4 h-4 rounded-full ${mode.icon} ring-4 ${mode.ring}`} />
              <span className="text-xl font-semibold text-gray-700">{mode.text}</span>
            </div>
          </div>
        </div>

        <div className="bg-white rounded-xl shadow-sm border border-gray-100 p-5">
          <div className="text-xs font-medium text-gray-500 uppercase tracking-wider">GPS</div>
          {data?.gps_valid ? (
            <div className="mt-1 text-lg font-mono text-gray-700">
              {Number(data.lat).toFixed(6)}, {Number(data.lng).toFixed(6)}
            </div>
          ) : (
            <div className="mt-1 text-gray-400">No GPS fix</div>
          )}
        </div>

        <div className="text-center text-xs text-gray-400">
          Last update: {timeAgo()}
        </div>
      </main>
    </div>
  );
}
