import StatCard from '../components/StatCard';

const Analytics = () => {
  return (
    <div className="p-6 space-y-6">
      <h1 className="text-2xl font-semibold">Welcome to AGV Dashboard</h1>

      <div className="grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 gap-4">
        <StatCard title="Active Users" value="1,204" />
        <StatCard title="Monthly Revenue" value="$12,000" />
        <StatCard title="Conversion Rate" value="4.2%" />
      </div>
    </div>
  );
};

export default Analytics;