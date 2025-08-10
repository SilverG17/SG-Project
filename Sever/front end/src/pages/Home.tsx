import StatCard from '../components/StatCard';

const Home = () => (
  <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
    <StatCard title="Users" value="1,024" />
    <StatCard title="Revenue" value="$12,340" />
    <StatCard title="Orders" value="320" />
  </div>
);

export default Home;